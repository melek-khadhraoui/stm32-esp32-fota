# STM32F401RE – ESP32 FOTA System (Jenkins-Driven)

A robust Firmware Over-The-Air (FOTA) update system designed for embedded devices using an **STM32F401RE** microcontroller with an **ESP32** acting as a Wi-Fi gateway.
The update process is automated via Jenkins, which responds to binary file pushes to GitHub, calculates the CRC32 checksum, and publishes MQTT notifications to trigger the ESP32 to download and transfer firmware. 
The STM32 validates the firmware’s integrity before flashing, ensuring secure and reliable updates.

---

## Features

- Automated Jenkins pipeline triggered by GitHub binary pushes that calculates CRC32 and sends MQTT update messages.
- MQTT-based signaling using Mosquitto broker for update notifications.
- ESP32 gateway downloads firmware over Wi-Fi and streams it via UART to STM32.
- STM32 performs CRC32 verification on received firmware before flashing.
- Secure, reliable firmware updates without physical access.
- Supports integration with existing GitHub workflows and CI/CD pipelines.

---

## Hardware Requirements

- STM32F401RE Nucleo board (or equivalent STM32F4 series)
- ESP32 development board with Wi-Fi capability
- Mosquitto MQTT broker (local or cloud)
- USB cables and jumpers for UART and programming
- ST-Link or compatible programmer for STM32
- Power supply for boards (usually 5V)

---

## How It Works

1. **Binary Push to GitHub**  
   The developer builds the STM32 firmware binary (`new_application.bin`) locally or in any environment and pushes it to a dedicated GitHub repository (e.g., my Fota Project repo).

2. **Jenkins Pipeline Trigger**  
   Jenkins monitors the GitHub repository via webhook.  
   When a new binary is pushed, Jenkins:  
   - Downloads the `.bin` file.  
   - Calculates the CRC32 checksum using `crc_script.py` or equivalent.  
   - Publishes two MQTT messages to the Mosquitto broker:  
     - Firmware availability (`fota/firmware/status: "available"`)  
     - CRC checksum (`fota/firmware/crc: e.g., "FED78FB8"`)  
   - Uses **ngrok** to expose the local Jenkins server securely over the internet, enabling GitHub webhook triggers during the FOTA process.

3. **ESP32 Receives MQTT Notification**  
   ESP32 subscribes to the relevant MQTT topics.  
   On receiving `"available"`, ESP32 fetches the new firmware `.bin` from GitHub raw URL or another HTTP server.

4. **ESP32 Transfers Firmware via UART**  
   ESP32 streams the downloaded firmware in chunks over UART to STM32.

5. **STM32 Performs CRC Verification**  
   STM32 computes CRC32 on the received data.  
   Compares it to the CRC sent by Jenkins (relayed via ESP32 or MQTT).  
   Only if CRC matches, STM32 writes the firmware to flash memory and safely reboots into the updated firmware.

---

## Jenkins Pipeline Overview

- **Trigger:** Push binary firmware `.bin` file to GitHub  
- **Steps:**  
  - Retrieve latest `.bin` from GitHub repo.
  - Calculate CRC32 checksum on `.bin` using the included `crc_script.py` Python script for consistent version-controlled checksum calculation.  
  - Publish MQTT messages signaling firmware availability and CRC.
<img width="1231" height="507" alt="Capture d'écran 2025-08-09 204422" src="https://github.com/user-attachments/assets/97351ccd-85a7-4bf5-88b1-ed763f156c9a" />

---

## MQTT Topics and Messages

| Topic                  | Payload    | Description                           |
|------------------------|------------|-------------------------------------|
| `fota/firmware/status` | `available`| Signals new firmware availability   |
| `fota/firmware/crc`    | `FED78FB8` | CRC32 checksum (hex) for integrity  |
| `update_status`        | `success`  | Indicates firmware successfully sent|

<img width="261" height="104" alt="Capture d'écran 2025-08-09 204726" src="https://github.com/user-attachments/assets/48545650-4b23-4a45-844c-2db66cb6d8d0" />

---

## STM32 Firmware Responsibilities

- Receive firmware bytes over UART from ESP32 and write them to a dedicated update memory area (not main application flash).
- Calculate CRC32 checksum on the entire received firmware in the update area.
- Compare calculated CRC with the Jenkins-published checksum (received via ESP32 or MQTT).
- Only if CRC matches, move the verified firmware to application flash, replacing the running firmware.
- If CRC mismatch, discard the update to prevent bricking.
- Safely reboot into the updated application after flashing.
- Optionally provide status feedback (UART, GPIO, MQTT) about update success or failure.

---

## ESP32 Firmware Responsibilities

- Connect to Wi-Fi and MQTT broker.
- Subscribe to firmware update MQTT topics.
- On receiving `"available"` status:  
  - Download new firmware `.bin` from HTTP server or GitHub raw URL.  
  - Stream firmware over UART to STM32 in proper chunk sizes.

---
---

## Demo

Check out this quick demo of the FOTA update process in action:

(https://www.youtube.com/shorts/dUoxlsj4Zmo)

---


## Future Perspective: Secure Firmware Update with Signed Header

### Why add a signed firmware header?

In any Firmware Over-The-Air (FOTA) update system, **security and integrity** are crucial. Without proper protection, attackers could:

- Inject malicious firmware,
- Corrupt the update process,
- Or cause devices to malfunction or become unusable ("bricked").

To prevent these risks, a **cryptographically signed firmware header** can be added to each firmware update. This ensures the device:

- Only accepts firmware signed by a trusted source,
- Verifies the integrity of the entire firmware image,
- Protects against unauthorized or corrupted updates.

---

### Firmware Header Structure

```c
#pragma pack(push,1)
typedef struct {
    uint32_t magic;          // Fixed identifier: 0xF07A10AD
    uint32_t fw_version;     // Firmware version number
    uint32_t fw_size;        // Firmware payload size in bytes
    uint8_t  fw_sha256[32];  // SHA-256 hash of the firmware payload
    uint8_t  pubkey_id;      // Public key ID for signature verification
    uint8_t  reserved[15];   // Padding for 64 bytes before signature
    uint8_t  sig[64];        // Ed25519 signature over (header_without_sig || firmware payload)
} fota_hdr_t;
#pragma pack(pop)

### How the System Works

#### Jenkins Build Server

- Compiles the firmware binary.
- Calculates the SHA-256 hash of the firmware.
- Builds the header with all metadata except the signature.
- Signs the concatenation of the header (without `sig`) and the firmware payload using a private Ed25519 key stored securely in Jenkins.
- Inserts the signature into the header.
- Publishes the signed firmware (header + payload) for distribution.

#### STM32 Device

- Receives the signed firmware over UART from ESP32.
- Parses and validates the header’s `magic` number and firmware version.
- Computes SHA-256 hash of the received payload and compares it with `fw_sha256`.
- Verifies the Ed25519 signature with the stored public key matching `pubkey_id`.
- Proceeds with flashing only if all validations succeed.
- Rejects the update if verification fails to avoid bricking.


---

*This project was created and contributed by **Melek Khadhraoui**,  
    a passionate Embedded Systems Engineering student.*

---

