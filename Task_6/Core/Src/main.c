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
#include "pca9685_pwm.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	DOWN,
	UP
} adjust;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PCA9685_PWM_LED_ID 0x80

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t ledOnTime = 0;
uint8_t ledOffTime = 0;
uint8_t ledChannel = 0;
uint8_t sleepMode = 0;
uint8_t pwmChannelsState = 0;
uint16_t pwmFrequency = 100;

char rcvBuf;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
static void adjustledOffTime(adjust offTime);
static void adjustledOnTime(adjust onTime);
static void adjustPwmFrequency(adjust freq);
static void toggleAllPwmOutputs(void);
static void toggleSleepMode(void);
static void disablePwmOutput(uint8_t ledChannel);
static void receiveUartStatus(void);
static void transmitUartMessage(char terminalMessage[256]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Pooling UART using Timer3 every 0.02 seconds
	if (htim == &htim3 )
	{

		HAL_StatusTypeDef result = HAL_UART_Receive(&huart3, (uint8_t*)&rcvBuf, 1, 10);

		if (result == HAL_OK)
		{
			switch (rcvBuf)
			{
			case '1':
				adjustledOffTime(DOWN);
				break;
			case '3':
				adjustledOffTime(UP);
				break;
			case '4':
				adjustledOnTime(DOWN);
				break;
			case '6':
				adjustledOnTime(UP);
				break;
			case '7':
				adjustPwmFrequency(DOWN);
				break;
			case '9':
				adjustPwmFrequency(UP);
				break;
			case '2':
				toggleAllPwmOutputs();
				break;
			case '5':
				toggleSleepMode();
				break;
			case '8':
				disablePwmOutput(ledChannel);
				break;
			case '+':
				ledChannel = (ledChannel < 16) ? (ledChannel + 1) : ledChannel;
				break;
			case '-':
				ledChannel =( ledChannel < 0) ? ledChannel  : (ledChannel - 1) ;
				break;
			default:
				HAL_UART_Transmit(&huart3, (uint8_t*) "Unrecognized key\r\n", 15 + 2, 10);
				break;
			}
			receiveUartStatus();
		}
	}
}

static void adjustledOffTime(adjust offTime)
{

	if(offTime == DOWN)
	{
		ledOffTime = (ledOffTime > 0) ? (ledOffTime - 1) : ledOffTime;
		ledOffTime = (ledOffTime > (100 - ledOnTime)) ? (100 - ledOnTime) : ledOffTime;

	}
	else if (offTime == UP)
	{
		ledOffTime = (ledOffTime < 100) ? (ledOffTime + 1) : ledOffTime;
		ledOffTime = (ledOffTime > (100 - ledOnTime)) ? (100 - ledOnTime) : ledOffTime;
	}

	if (pwmChannelsState == 1)
	{
		PWM_Register_All_Set(&hi2c1, PCA9685_PWM_LED_ID, ledOnTime, ledOffTime);
	}
	else
		PWM_Register_Set(&hi2c1, PCA9685_PWM_LED_ID, ledOnTime, ledOffTime, ledChannel);
}

static void adjustledOnTime(adjust onTime)
{
	if(onTime == DOWN)
	{
		ledOnTime = (ledOnTime > 0) ? (ledOnTime - 1) : ledOnTime;
		ledOnTime = (ledOnTime > (100 - ledOffTime)) ? (100 - ledOffTime) : ledOnTime;
	}
	else if (onTime == UP)
	{
		ledOnTime = (ledOnTime < 100) ? (ledOnTime + 1) : ledOnTime;
		ledOnTime = (ledOnTime > (100 - ledOffTime)) ? (100 - ledOffTime) : ledOnTime;
	}
	if (pwmChannelsState == 1)
	{
		PWM_Register_All_Set(&hi2c1, PCA9685_PWM_LED_ID, ledOnTime, ledOffTime);
	}
	else
		PWM_Register_Set(&hi2c1, PCA9685_PWM_LED_ID, ledOnTime, ledOffTime, ledChannel);
}

static void adjustPwmFrequency(adjust freq)
{

	if(freq == DOWN)
	{
		pwmFrequency = (pwmFrequency > 24) ? (pwmFrequency - 1) : pwmFrequency;
	}
	else if (freq == UP)
	{
		pwmFrequency = (pwmFrequency < 1526) ? (pwmFrequency + 1) : pwmFrequency;
	}

	PWM_Frequency_Set(&hi2c1, PCA9685_PWM_LED_ID, pwmFrequency);

}

static void toggleAllPwmOutputs(void)
{
	if(pwmChannelsState == 0)
	{
		pwmChannelsState = 1;
		PWM_Register_All_Set(&hi2c1, PCA9685_PWM_LED_ID, ledOnTime, ledOffTime);
	}
	else
	{
		pwmChannelsState = 0;
		PWM_Register_All_Off(&hi2c1, PCA9685_PWM_LED_ID);
	}

}

static void disablePwmOutput(uint8_t ledChannel)
{
	PWM_Register_Off(&hi2c1, PCA9685_PWM_LED_ID, ledChannel);
}

static void toggleSleepMode(void)
{
	sleepMode = !sleepMode;

	PWM_Sleep_State(&hi2c1, PCA9685_PWM_LED_ID, sleepMode);

}

static void receiveUartStatus(void)
{
	/** the ASCII Escape character, value 0x1B.
        the ASCII left square brace character, value 0x5B.
      	  the ASCII character for numeral 2, value 0x32.
        the ASCII character for the letter J, value 0x4A.
        Esc[2J - erase terminal display
 	*/
	static char tempStr[256];
	uint8_t  eraseScreenArr[5] ={ 0x1B, 0x5B, 0x32, 0x4A };

	HAL_UART_Transmit(&huart3, (uint8_t*) &eraseScreenArr, 4, 10);

	transmitUartMessage("Duty cycle: Decrease/Increase 1/3 | Delay time: Decrease/Increase 4/6 | PWM Frequency: Decrease/Increase 7/9 | Specific Channel: Decrease/Increase -/+ |\r\n"
			"Toggle all channels: 2 | Toggle sleep mode: 5 | Disable specific channel: 8\r\n");

	snprintf(tempStr, sizeof(tempStr), "\r\nDuty Cycle: %u %%\r\nDelay: %u %%\r\nFrequency: %u Hz\r\nAll channels: %u\r\nSleep mode: %u\r\nChannel: %u\r\n",
	(unsigned int)(ledOffTime), (unsigned int)(ledOnTime), (unsigned int)(pwmFrequency),(unsigned int)(pwmChannelsState),(unsigned int)(sleepMode),(unsigned int)(ledChannel));

	transmitUartMessage(tempStr);
}

static void transmitUartMessage(char terminalMessage[256])
{
	HAL_UART_Transmit(&huart3, (uint8_t*) terminalMessage,  strlen(terminalMessage), 10);
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  PWM_LED_Init(&hi2c1, PCA9685_PWM_LED_ID, GPIOB, GPIO_PIN_7);
  HAL_TIM_Base_Start_IT(&htim3);
  receiveUartStatus();
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim3.Init.Prescaler = 1600;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
