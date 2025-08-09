/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void(*ptrFunc)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 We will actually be dividing our flash to 3 parts:
 bootloader space  0x08000000—0x08007FFF (32KB )
 application space 0x08008000-0x0803FFFF (224KB)
 update space      0x08040000-0x0807FFFF (256KB)
 */
#define CHUNK_SIZE 1024*15 //15KB
#define APPLICATION_ADRESS 0x08008000
#define UPDATE_SPACE_ADRESS 0x08040000
#define VALID_FIRMWARE_FLAG_ADRESS 0x0807FFF0
#define FIRMWARE_SIZE_ADRESS 0x0807FFFC
#define RECEIVED_CRC_ADRESS 0x0807FFF8


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t buffer[CHUNK_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void ReceiveFirmawre(void);
uint8_t ValidateFirmware(void);
void WriteFirmware(void);
void JumpToApplication(void);
void CheckAndJumpIfFirmwareValid(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ReceiveFirmawre(void){
	uint32_t firmwareSize;
	uint32_t firmwareCRC;
	if(HAL_UART_Receive(&huart1, &firmwareSize, sizeof(firmwareSize), 7000)!=HAL_OK)
	{HAL_UART_Transmit(&huart2, "NO CODE \n", strlen("NO CODE \n"), 10);
		JumpToApplication();}
	//firmwareSize *= 1024;
	if(HAL_UART_Receive(&huart1, &firmwareCRC, sizeof(firmwareCRC), 2000)!=HAL_OK)
		JumpToApplication();
	char debugMsg[50];
	int len = snprintf(debugMsg, sizeof(debugMsg), "Firmware size: %lu bytes\n", firmwareSize);
	HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, len, HAL_MAX_DELAY);

	if(HAL_UART_Receive(&huart1, buffer,firmwareSize,20000)!=HAL_OK)
		{HAL_UART_Transmit(&huart2, "NO FIRMWARE RECEIVED \n", strlen("NO FIRMWARE RECEIVED \n"), 100);
		JumpToApplication();}
	HAL_UART_Transmit(&huart2, " FIRMWARE RECEIVED \n", strlen(" FIRMWARE RECEIVED \n"), 100);
	FLASH_EraseInitTypeDef EraseStructure;
	uint32_t SectorError = 0;
	EraseStructure.Banks=FLASH_BANK_1;
	EraseStructure.NbSectors=2;
	EraseStructure.Sector=FLASH_SECTOR_6;
	EraseStructure.TypeErase=FLASH_TYPEERASE_SECTORS;
	EraseStructure.VoltageRange=FLASH_VOLTAGE_RANGE_3;
	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&EraseStructure, &SectorError) != HAL_OK) {
		HAL_FLASH_Lock();
		return;
	}
	for(uint32_t i=0 ; i<firmwareSize/4 ; i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)(UPDATE_SPACE_ADRESS+ i*4),((uint32_t*)buffer)[i]);
	}
	uint8_t RemainingBytes = firmwareSize % 4;
	if(RemainingBytes>0)
	{
		uint32_t lastword=0xFFFFFFFF;
		memcpy(&lastword,buffer + firmwareSize - RemainingBytes,RemainingBytes);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)UPDATE_SPACE_ADRESS+((firmwareSize/4)*4) , lastword);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)(FIRMWARE_SIZE_ADRESS),firmwareSize);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)(RECEIVED_CRC_ADRESS),firmwareCRC);
	HAL_FLASH_Lock();
	HAL_UART_Transmit(&huart2, " FIRMWARE COPIED TO UPDATE \n", strlen(" FIRMWARE COPIED TO UPDATE \n"), 100);
}

uint8_t ValidateFirmware(void) {
    char msg[100];


    uint32_t firmwareSize = *((uint32_t*)FIRMWARE_SIZE_ADRESS);
  hcrc.Instance->CR |= CRC_CR_RESET;

    // Compute CRC over all full 32-bit words
    uint32_t CalculatedCRC = HAL_CRC_Calculate(&hcrc, (uint32_t*)UPDATE_SPACE_ADRESS, firmwareSize / 4);
    uint8_t RemainingBytes = firmwareSize % 4;
    if (RemainingBytes > 0) {
    	uint32_t lastWord = 0;
    	memcpy(&lastWord, (uint8_t*)(UPDATE_SPACE_ADRESS + firmwareSize - RemainingBytes), RemainingBytes);
    	CalculatedCRC = HAL_CRC_Accumulate(&hcrc, &lastWord, 1);
    }
    uint32_t ReceivedCRC = *((uint32_t*)RECEIVED_CRC_ADRESS);

    // Log CRCs
    snprintf(msg, sizeof(msg), "Calculated CRC: 0x%08lX, Received CRC: 0x%08lX\n", (unsigned long)CalculatedCRC, (unsigned long)ReceivedCRC);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    if (CalculatedCRC == ReceivedCRC) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"FIRMWARE VALIDATED\n", strlen("FIRMWARE VALIDATED\n"), 100);
        return 1;
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"FIRMWARE VALIDATION FAILED\n", strlen("FIRMWARE VALIDATION FAILED\n"), 100);
        return 0;
    }
}


void WriteFirmware(void){
	if(!ValidateFirmware())
	{ JumpToApplication();
        return;}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, VALID_FIRMWARE_FLAG_ADRESS, 0xFFFFFFFF);
	FLASH_EraseInitTypeDef EraseStructure;
	uint32_t SectorError = 0;
	EraseStructure.Banks=FLASH_BANK_1;
	EraseStructure.NbSectors=4;
	EraseStructure.Sector=FLASH_SECTOR_2;
	EraseStructure.TypeErase=FLASH_TYPEERASE_SECTORS;
	EraseStructure.VoltageRange=FLASH_VOLTAGE_RANGE_3;
	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&EraseStructure, &SectorError) != HAL_OK) {
		HAL_FLASH_Lock();
		return;
	}
	uint32_t firmwareSize=*((uint32_t*)FIRMWARE_SIZE_ADRESS);
	for(uint32_t i=0 ; i<firmwareSize/4 ; i++){

		uint32_t writeAddr = APPLICATION_ADRESS + i * 4;
		uint32_t data = *((uint32_t*)(UPDATE_SPACE_ADRESS + i * 4));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, writeAddr, data);

	}
	uint8_t RemainingBytes = firmwareSize % 4;
	if(RemainingBytes>0)
	{
		uint32_t lastword=0xFFFFFFFF;
		memcpy(&lastword,  (uint8_t*)(UPDATE_SPACE_ADRESS + firmwareSize - RemainingBytes),RemainingBytes);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)APPLICATION_ADRESS+((firmwareSize/4)*4) , lastword);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, VALID_FIRMWARE_FLAG_ADRESS, 0x00000000);
	HAL_FLASH_Lock();
	HAL_UART_Transmit(&huart2, (uint8_t*)"FIRMWARE WRITING\n", strlen("FIRMWARE WRITING\n"), 100);

}

void JumpToApplication(void)
{
    //char msg[100];
	uint32_t MSP_Value = *((volatile uint32_t*)APPLICATION_ADRESS);
	uint32_t AppJump = *((volatile uint32_t*)(APPLICATION_ADRESS+4));
   // snprintf(msg, sizeof(msg), "MSP VALUE: %d", MSP_Value);
    //HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	if ((MSP_Value >= 0x20000000) && (MSP_Value <= 0x2001FFFF))
	{
		HAL_UART_Transmit(&huart2, "JUMPING TO APP \n", strlen("JUMPING TO APP \n"), 10);
		HAL_DeInit();
		HAL_RCC_DeInit();

		HAL_Delay(10);             // Small delay to ensure completion
		__set_MSP(MSP_Value);

		ptrFunc Jump=(ptrFunc)AppJump;
		Jump();
	}
	//else
		//HAL_UART_Transmit(&huart2, "INVALID FIRMWARE \n", strlen("INVALID FIRMWARE \n"), 10);
}

void CheckAndJumpIfFirmwareValid(void)
{
    uint32_t flag = *((volatile uint32_t*)VALID_FIRMWARE_FLAG_ADRESS);

    if (flag == 0x00000000)
    {
        JumpToApplication();
    }
    else
    {
        HAL_UART_Transmit(&huart2, "INVALID FIRMWARE\r\n", strlen("INVALID FIRMWARE \n"), HAL_MAX_DELAY);

    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_PWR_EnableBkUpAccess();
  if(RTC->BKP1R!=0xF07AF07A)
	  JumpToApplication();
  RTC->BKP1R=0xFFFFFFFF;//clear the flag
  ReceiveFirmawre();
  WriteFirmware();
  CheckAndJumpIfFirmwareValid();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
