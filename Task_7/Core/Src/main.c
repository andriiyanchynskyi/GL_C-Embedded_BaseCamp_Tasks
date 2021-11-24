/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum chipState
{
	CS_ON,
	CS_OFF

} chipState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CS_PIN 	 GPIOD, GPIO_PIN_7
#define SST25_PageSize 256

#define WRITE    0x02  /* Write one data byte to Memory instruction */
#define WREN 	 0x06  /* Write enable instruction */
#define WRDI	 0x04  /* Write disable instruction */
#define RDSR	 0x05  /* Read Status Register instruction  */
#define READ     0x03  /* 25Mhz Read from Memory instruction */
#define HREAD    0x0B  /* 50Mhz Read from Memory instruction */
#define WRSR     0x01  /* Write Status Register instruction */
#define CHER     0x60  /* Bulk Erase instruction */
#define EWSR	 0x50  /* Enable write status register */
#define D_BYTE   0xFF  /* Dummy byte */
#define EBSY	 0x70
#define DBSY	 0x80
#define AAI	 	 0xAD

#define MIN_ADDR 0x000000
#define MAX_ADDR 0x1FFFFF
#define SEC_INC  0x1000 /* 4096Bytes Sector increment   */
#define BUSY_BIT 0x01

#define STR_NUM  16
#define STR_LEN  96


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char rcvCommand = 0;
static uint8_t trnsmtBuf[2];
static uint8_t rcvBuf[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void SST25Init(void);
static void SST25CSToggle(chipState state);
static void SST25RegSwitch(uint8_t trnsmtBuf);
static uint8_t SST25GetRdStatus(void);
static void SST25RdBuffer(uint32_t addr, uint8_t *buf, uint32_t nByte );
static void SST25WrBuffer(uint32_t dst, uint8_t *data, uint32_t nByte);
static void SST25WrPage(uint32_t dst, uint8_t *data, uint32_t nByte);
static void SST25WrStatusReg(void);
static void SST25EraseAll(void);

static void transmitUartMessage(char terminalMessage[128]);
static void receiveInfo(void);
static void writeText(void);
static void readText(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3 )
	{
		HAL_StatusTypeDef result = HAL_UART_Receive(&huart3, (uint8_t*)&rcvCommand, 1, 10);

		if (result == HAL_OK)
		{
			receiveInfo();
			switch (rcvCommand)
			{
			case '1':
				readText();
				break;
			case '2':
				writeText();
				break;
			case '3':
				SST25EraseAll();
				break;
			default:
				HAL_UART_Transmit(&huart3, (uint8_t*) "Unrecognized key\r\n", 15 + 2, 10);
				break;
			}
		}
	}
}

static void transmitUartMessage(char terminalMessage[128])
{
	HAL_UART_Transmit(&huart3, (uint8_t*) terminalMessage,  strlen(terminalMessage), 10);
}

static void receiveInfo(void)
{
	uint8_t  eraseScreenArr[4] ={ 0x1B, 0x5B, 0x32, 0x4A };
	HAL_UART_Transmit(&huart3, (uint8_t*) &eraseScreenArr, 4, 10);

	transmitUartMessage("Read SST25VF016 SPI Flash Memory: 1 | Write text to SST25VF016 SPI Flash Memory: 2 | Erase SST25VF016 SPI Flash Memory: 3\r\n");
}

static void SST25Init(void)
{
	/* CS = HIGH */
	SST25CSToggle(CS_OFF);
	SST25WrStatusReg();
}

static void SST25CSToggle(chipState state)
{
	if(state == CS_ON)
		HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_RESET);
	else if (state == CS_OFF)
		HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_SET);
}

static void SST25RegSwitch(uint8_t trnsmtBuf)
{
	SST25CSToggle(CS_ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t *) &trnsmtBuf, sizeof(trnsmtBuf), 100);
	SST25CSToggle(CS_OFF);
}

static uint8_t SST25GetRdStatus(void)
{
	trnsmtBuf[0] = RDSR;
	trnsmtBuf[1] = 0x00;

	SST25CSToggle(CS_ON);
	HAL_SPI_TransmitReceive(&hspi1, trnsmtBuf, rcvBuf, sizeof(rcvBuf), 100);
	SST25CSToggle(CS_OFF);

	return  rcvBuf[1];
}

static void SST25WrStatusReg(void)
{
	/* Disable write protection */
	trnsmtBuf[0] = WRSR;
	trnsmtBuf[1] = 0x00;

	SST25RegSwitch(EWSR);

	SST25CSToggle(CS_ON);
	HAL_SPI_Transmit(&hspi1, trnsmtBuf, sizeof(trnsmtBuf), 100);
	SST25CSToggle(CS_OFF);
}

static void SST25EraseAll(void)
{
	uint8_t cmdBuf = CHER;

	SST25WrStatusReg();
	SST25RegSwitch(WREN);
	SST25CSToggle(CS_ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &cmdBuf, sizeof(cmdBuf), 100);
	SST25CSToggle(CS_OFF);

	while(SST25GetRdStatus() & BUSY_BIT);
	SST25RegSwitch(WRDI);
}

static void SST25RdBuffer(uint32_t addr, uint8_t *buf, uint32_t nByte)
{
	if (addr + nByte > MAX_ADDR)
		return;

	uint8_t tmpBuf[nByte];
	uint8_t trnsmtRdBuf[] = { HREAD, ((addr  & 0xFF0000) >> 16), ((addr  & 0xFF00) >> 8), addr & 0xFF, D_BYTE };

	SST25CSToggle(CS_ON);
	HAL_SPI_Transmit(&hspi1, trnsmtRdBuf, sizeof(trnsmtRdBuf), 100);
	HAL_SPI_Receive(&hspi1, tmpBuf, sizeof(tmpBuf) , 100 );
	for (uint8_t i = 1; i < nByte; i++)
		buf[i] = 0;
	for ( uint32_t i = 1; tmpBuf[i] != 0xFF; i++ )
		buf[i] = tmpBuf[i];
	SST25CSToggle(CS_OFF);
}

static void SST25WrPage(uint32_t dst, uint8_t *data, uint32_t nByte)
{
	if (dst + nByte - 1 > MAX_ADDR || nByte > SST25_PageSize )
			return;

	SST25WrStatusReg();

	uint8_t *trnsmtBufPt;
	trnsmtBufPt = data;

	SST25RegSwitch(EBSY);
	SST25RegSwitch(WREN);

	SST25CSToggle(CS_ON);
	/* Transmit address and first two bytes of data */
	uint8_t trnsmtPgBuf[] = { AAI, ((dst  & 0xFF0000) >> 16), ((dst  & 0xFF00) >> 8), (dst & 0xFF), D_BYTE, *data,*data++ };
	HAL_SPI_Transmit(&hspi1, trnsmtPgBuf, sizeof(trnsmtPgBuf), 100);

	while(data < trnsmtBufPt + nByte)
	{
		/* Check for flash busy status */
		SST25CSToggle(CS_OFF);
		SST25CSToggle(CS_ON);
		while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4));
		/* Transmit data using AAI */
		uint8_t trnsmtPgBuf[] = { AAI, *data++, *data++ };
		HAL_SPI_Transmit(&hspi1, trnsmtPgBuf, sizeof(trnsmtPgBuf), 100);
	}
	SST25CSToggle(CS_OFF);
	while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4));

	SST25RegSwitch(WRDI);
	SST25RegSwitch(DBSY);
}

static void SST25WrBuffer(uint32_t dst, uint8_t *data, uint32_t nByte)
{
	if (dst + nByte - 1 > MAX_ADDR)
			return;

	SST25WrStatusReg();
	uint8_t *trnsmtBufPt = (uint8_t *)data;

	for ( uint32_t i = 0; i < nByte; i++ )
	{
		SST25RegSwitch(WREN);

		SST25CSToggle(CS_ON);
		uint8_t trnsmtWrBuf[] = { WRITE, (((dst + i) & 0xFF0000) >> 16), (((dst + i) & 0xFF00) >> 8), ((dst + i) & 0xFF), D_BYTE, trnsmtBufPt[i] };
		HAL_SPI_Transmit(&hspi1, trnsmtWrBuf, sizeof(trnsmtWrBuf), 100);
		SST25CSToggle(CS_OFF);

		while(SST25GetRdStatus() & BUSY_BIT);
	}
	SST25RegSwitch(WRDI);
}

static void writeText(void)
{
	SST25EraseAll();
	uint32_t addr = MIN_ADDR;

	char capsuleText [STR_NUM][STR_LEN] =
	{
			"From: Andrii Yanchynskyi, andrii.yanchynskyi@gmail.com\r\n",
			"Mentor: Denys Kondratenko, denys.kondratenko@globallogic.com\r\n",
			"Date: 24.11.2021\r\n",
			"TIME CAPSULE\r\n",
			"System Of A Down - Aerials\r\n",
			"Life is a waterfall\r\n",
			"We're one in the river\r\n",
			"And one again after the fall\r\n",
			"Swimming through the void, we hear the word\r\n",
			"We lose ourselves, but we find it all\r\n",
			"Cause we are the ones that want to play\r\n",
			"Always want to go, but you never want to stay\r\n",
			"And we are the ones that want to choose\r\n",
			"Always want to play but you never want to lose\r\n",
			"Aerials in the sky\r\n",
			"When you lose small mind, you free your life\r\n",
	};

	for (uint8_t i = 0; i < STR_NUM || addr > MAX_ADDR; i++, addr += SEC_INC)
	{
		char tmpStr[] = {};
		strcpy(tmpStr, &capsuleText[i][0]);
		SST25WrPage(addr, (uint8_t*) tmpStr, strlen(tmpStr));
	}
}

static void readText(void)
{
	uint32_t  addr = MIN_ADDR;
	char tmpRdStr[STR_LEN];

  	for (uint8_t i = 0; i < STR_NUM || addr > MAX_ADDR; i++, addr += SEC_INC)
  	{
  		SST25RdBuffer(addr, (uint8_t *) tmpRdStr, STR_LEN);
  		transmitUartMessage(tmpRdStr);
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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE BEGIN 2 */
  SST25Init();
  receiveInfo();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
