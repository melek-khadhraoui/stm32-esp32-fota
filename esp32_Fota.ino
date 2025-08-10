#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>

const char* ssid = "";
const char* password = "";

const char* mqtt_server = "test.mosquitto.org";

const char* firmwareURL = "https://raw.githubusercontent.com/melek-khadhraoui/Fota-Project/main/new_application.bin";

HardwareSerial& uartToSTM = Serial2;

WiFiClient espClient;
PubSubClient client(espClient);

bool firmwareAvailable = false;
String receivedCRC = "";

void callback(char* topic, byte* payload, unsigned int length) {
  String message;

  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == "fota/firmware/status") {
    if (message == "available") {
      firmwareAvailable = true;
      Serial.println("Firmware available signal received.");
      client.subscribe("fota/firmware/crc");
    } else if (message == "done") {
      firmwareAvailable = false;
      receivedCRC = "";
      Serial.println("Firmware update done signal received.");
    }
  } else if (String(topic) == "fota/firmware/crc") {
    receivedCRC = message;
    Serial.print("Received CRC: ");
    Serial.println(receivedCRC);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32_FOTA_Client")) {
      Serial.println("connected");
      client.subscribe("fota/firmware/status");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  uartToSTM.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(21, OUTPUT);     // Configure GPIO21 en sortie
  digitalWrite(21, LOW);   // Initialement à LOW

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  reconnect();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (firmwareAvailable && receivedCRC.length() > 0) {
    Serial.println("Starting firmware download and update...");
    if (downloadAndSendFirmware()) {
      Serial.println("Firmware sent successfully.");
      client.publish("fota/firmware/status", "done");
      client.publish("fota/firmware/update_status", "success");
    } else {
      Serial.println("Failed to send firmware.");
      client.publish("fota/firmware/update_status", "failure");
    }

    firmwareAvailable = false;
    receivedCRC = "";
  }

  delay(100);
}

bool downloadAndSendFirmware() {
  HTTPClient http;
  http.begin(firmwareURL);
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("HTTP GET failed, error: %d\n", httpCode);
    http.end();
    return false;
  }

  int len = http.getSize();
  Serial.printf("Firmware size: %d bytes\n", len);

  WiFiClient* stream = http.getStreamPtr();

  uint8_t* firmwareData = (uint8_t*)malloc(len);
  if (!firmwareData) {
    Serial.println("Failed to allocate memory");
    http.end();
    return false;
  }

  int bytesRead = 0;
  while (http.connected() && bytesRead < len) {
    int avail = stream->available();
    if (avail > 0) {
      int r = stream->readBytes(firmwareData + bytesRead, avail);
      bytesRead += r;
    }
    delay(1);
  }

  // Convert received CRC from string to uint32_t
  uint32_t crc = strtoul(receivedCRC.c_str(), NULL, 16);
  Serial.printf("Using CRC from MQTT: 0x%08X\n", crc);

  digitalWrite(21, HIGH);    // Signal start update via pin 21
  Serial.println("Pin 21 set HIGH to signal start update");
  delay(1000);               // Delay pour laisser le temps au STM32

  Serial.println("Sending firmware size and CRC...");
  uartToSTM.write((uint8_t*)&len, 4);
  delay(10);

  uartToSTM.write((uint8_t*)&crc, 4);
  delay(10);

  // Send firmware
  uartToSTM.write(firmwareData, len);

  free(firmwareData);


  digitalWrite(21, LOW);     // Remettre la pin à LOW après envoi
  Serial.println("Pin 21 set LOW after sending firmware");

  http.end();

  return true;
}
