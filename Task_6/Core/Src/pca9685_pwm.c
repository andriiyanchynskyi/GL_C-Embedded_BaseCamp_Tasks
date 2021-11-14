/*
 * pca9685_pwm.c
 *
 *  Created on: 12 лист. 2021 р.
 *      Author: rozen
 */


/* Includes ------------------------------------------------------------------*/
#include "pca9685_pwm.h"

/* Defines  ------------------------------------------------------------------*/
#define INTERNAL_CLOCK_FREQ 25000000

/* Private macro -------------------------------------------------------------*/
#define USER_SET_BIT(p, n) ((p) | (n))
#define LED_OFF_TIME(a, b) ((a) + (int)((((b) * 4096) / 100) - 1))
#define PWM_FREQUENCY(f) (int)(((INTERNAL_CLOCK_FREQ) / (4096 * (f))) - 1)

/* Private variables ---------------------------------------------------------*/
static uint8_t clearData = 0x00;
static uint8_t allLedArray[4] = { ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H };

/* Private user code ---------------------------------------------------------*/

/**
 * This function transmits data to PCA9685's register address using I2C bus
 */
static void PWM_Data_Transmit(I2C_HandleTypeDef *hi2c, uint8_t devId, uint8_t regAddress, uint8_t data)
{
	uint8_t transmitBuf[] = { regAddress, data };

	HAL_I2C_Master_Transmit(hi2c, devId, (uint8_t*) &transmitBuf, 2, 1000);
}

/**
 * @brief Initializes the PCA9685 PWM LED Controller
 * This function disables low power mode and enables OE (Output Enable Pin)
 * @param  hi2c I2Cn &handle
 * @param  devId Developer address of the PWM LED controller
 * @retval None
 */
void PWM_LED_Init(I2C_HandleTypeDef *hi2c, uint8_t devId)
{
	static uint8_t setData = 0x00;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

	setData = USER_SET_BIT(setData, MODE1_LED_ALLCALL); // Set D0 bit in logic 1 to enable ALLCALL, disable low power mode by set D4 in logic 0
	PWM_Data_Transmit(hi2c, devId, MODE1_LED_REG, setData);

	HAL_Delay(10);
}

/**
 * @brief  This function disables all PWM LED registers
 * @param  hi2c I2Cn &handle
 * @param  devId Developer address of the PWM LED controller
 * @retval None
 */
void LED_Register_All_Off(I2C_HandleTypeDef *hi2c, uint8_t devId)
{
	for(uint8_t i = 0; i < 4; i++)
		PWM_Data_Transmit(hi2c, devId, allLedArray[i], clearData);
}

/**
 * @brief  This function is for enable or disable low power mode
 * @param  hi2c I2Cn &handle
 * @param  devId Developer address of the PWM LED controller
 * @param  state Low power mode flag, to wake up controller set flag to logic 1
 * @retval None
 */
void LED_Sleep_State(I2C_HandleTypeDef *hi2c, uint8_t devId, uint8_t state)
{
	static uint8_t setData = 0x00;

	if (state == 1)
	{
		setData = USER_SET_BIT(setData, MODE1_LED_ALLCALL);
		PWM_Data_Transmit(hi2c, devId, MODE1_LED_REG, setData);

		HAL_Delay(10);
	}
	else
	{
		setData = USER_SET_BIT(setData, MODE1_LED_SLEEP);
		PWM_Data_Transmit(hi2c, devId, MODE1_LED_REG, setData);
	}
}

/**
 * @brief  This function sets Duty Cycle and Delay time for all LED registers
 * @param  hi2c I2Cn &handle
 * @param  devId Developer address of the PWM LED controller
 * @param  delayTime Time of the delay: 0 - 100 %
 * @param  dutyCycle Duty cycle: 0 - 100 %
 * @retval None
 */
void LED_Register_All_Set(I2C_HandleTypeDef *hi2c, uint8_t devId, uint16_t delayTime, uint16_t dutyCycle)
{
	static uint32_t LED_ALL_ON;
	static uint32_t LED_ALL_OFF;

	if(delayTime < 0)
		LED_ALL_ON = 0;
	else
		LED_ALL_ON = (int)(delayTime * 4096) / 100;

	if(dutyCycle < 0)
		LED_ALL_OFF = 0;
	else
		LED_ALL_OFF = LED_OFF_TIME(LED_ALL_ON, dutyCycle);

	uint32_t transmitBuf[4] = {(LED_ALL_ON & 0xFF),(LED_ALL_ON >> 8),(LED_ALL_OFF & 0xFF),(LED_ALL_OFF >> 8)};

	for(uint8_t i = 0; i < 4; i++)
		PWM_Data_Transmit(hi2c, devId, allLedArray[i], transmitBuf[i]);
}

/**
 * @brief This function sets PWM frequency of the LEDs
 * The maximum PWM frequency is 1526 Hz if the PRE_SCALE register is set "0x03h"
 * The minimum PWM frequency is 24 Hz if the PRE_SCALE register is set "0xFFh"
 * The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1
 * Internal oscillator clock frequency - 25MHz
 * @param  hi2c I2Cn &handle
 * @param  devId Developer address of the PWM LED controller
 * @param  pwmFrequency Set frequency of the PWM LED controller: 24Hz - 1526Hz
 * @retval None
 */
void PWM_Frequency_Set(I2C_HandleTypeDef *hi2c, uint8_t devId, uint16_t frequency)
{
	static uint32_t PRE_SCALE;
	static uint8_t setData = 0x00;

	/*  Calculate frequency prescaler */
	if (frequency < 24)
		PRE_SCALE = 0xFF;
	else if (frequency > 1526)
		PRE_SCALE = 0x03;
	else
		PRE_SCALE = PWM_FREQUENCY(frequency);

 /* If the PCA9685 is operating and then sleep mode is enabled,
	RESTART bit (MODE1 bit 7) will be set to logic 1 at the end of the PWM refresh cycle.
	The contents of each PWM register are held valid when the clock is off. To restart PWM need to clear bit 4 (SLEEP).
	Allow time for oscillator to stabilize and write logic 1 to bit 7 of MODE1 register. */

	setData = USER_SET_BIT(setData, MODE1_LED_SLEEP);
	PWM_Data_Transmit(hi2c, devId, MODE1_LED_REG, setData);

	setData = 0x00;
	PWM_Data_Transmit(hi2c, devId, MODE1_LED_REG, setData);

	HAL_Delay(10);

	setData = USER_SET_BIT(USER_SET_BIT(setData, MODE1_LED_RESTART), MODE1_LED_ALLCALL );
	PWM_Data_Transmit(hi2c, devId, MODE1_LED_REG, setData);

}



