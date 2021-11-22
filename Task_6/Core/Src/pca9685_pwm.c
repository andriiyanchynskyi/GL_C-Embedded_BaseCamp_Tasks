/*
 * pca9685_pwm.c
 *
 *  Created on: 12 лист. 2021 р.
 *      Author: rozen
 */


/* Includes ------------------------------------------------------------------*/
#include "pca9685_pwm.h"

/* Typedefs ------------------------------------------------------------------*/
typedef struct
{
	uint32_t LEDn_ON;
	uint32_t LEDn_OFF;
} pwm_calc;

typedef struct
{
	uint8_t LEDn_ON_L;
	uint8_t LEDn_ON_H;
	uint8_t LEDn_OFF_L;
	uint8_t LEDn_OFF_H;
} ch_reg_calc;

/* Defines  ------------------------------------------------------------------*/
#define INTERNAL_CLOCK_FREQ 25000000
#define CLEAR_DATA 0x00

/* Private macro -------------------------------------------------------------*/
#define LED_REG_NUMBER 4
#define FREQUENCY_MIN 24
#define FREQUENCY_MAX 1526
#define PRE_SCALE_MIN 0xFF
#define PRE_SCALE_MAX 0x03
#define USER_SET_BIT(p, n) ((p) | (n))
#define LED_OFF_TIME(a, b) ((a) + (int)((((b) << 12) / 100) - 1))
#define PWM_FREQUENCY(f) (int)(((INTERNAL_CLOCK_FREQ) / ((f) << 12)) - 1)

/* Private variables ---------------------------------------------------------*/
static uint8_t allLedArray[LED_REG_NUMBER] = { ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H };
static uint8_t setData = CLEAR_DATA;

/* Private user code ---------------------------------------------------------*/

/**
 * This function transmits data to PCA9685's register address using I2C bus
 */
static void PWM_Data_Transmit(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint8_t data)
{
	uint8_t transmitBuf[] = { regAddress, data };

	HAL_I2C_Master_Transmit(hi2c, devAddress, (uint8_t*) &transmitBuf, 2, 1000);
}

/**
 * This function calculates LED on time and LED off time values
 */
pwm_calc PWM_Calc_Output(uint16_t ledOnTime, uint16_t ledOffTime)
{
	pwm_calc pwm;

	if(ledOnTime < 0)
		pwm.LEDn_ON = 0;
	else
		pwm.LEDn_ON = (int)(ledOnTime << 12) / 100; // Time of delay * 4096 / 100

	if(ledOffTime < 0)
		pwm.LEDn_OFF = 0;
	else
		pwm.LEDn_OFF = LED_OFF_TIME(pwm.LEDn_ON, ledOffTime);

	return pwm;
}

/**
 * This function sets PWM LED L/H registers by number of channel
 */
ch_reg_calc PWM_Set_Channel(uint8_t ledChannel)
{
	ch_reg_calc reg;

	reg.LEDn_ON_L  = (ledChannel << 2) + 6; // ledChannel * 4 + register definition offset
	reg.LEDn_ON_H  = (ledChannel << 2) + 7;
	reg.LEDn_OFF_L = (ledChannel << 2) + 8;
	reg.LEDn_OFF_H = (ledChannel << 2) + 9;

	return reg;
}
/**
 * @brief Initializes the PCA9685 PWM LED Controller
 * This function disables low power mode and enables OE (Output Enable Pin)
 * @param  hi2c I2Cn &handle
 * @param  devAddress Developer address of the PWM LED controller
 * @param  GPIOx Output enable port
 * @param  GPIO_Pin Output enable pin  number
 * @retval None
 */
void PWM_LED_Init(I2C_HandleTypeDef *hi2c, uint8_t devAddress, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

	HAL_Delay(10);

	setData = USER_SET_BIT(CLEAR_DATA, MODE1_LED_ALLCALL); // Set D0 bit in logic 1 to enable ALLCALL, disable low power mode by set D4 in logic 0
	PWM_Data_Transmit(hi2c, devAddress, MODE1_LED_REG, setData);
}

/**
 * @brief  This function disables all PWM LED registers
 * @param  hi2c I2Cn &handle
 * @param  devAddress Developer address of the PWM LED controller
 * @retval None
 */
void PWM_Register_All_Off(I2C_HandleTypeDef *hi2c, uint8_t devAddress)
{
	for(uint8_t i = 0; i < LED_REG_NUMBER; i++)
		PWM_Data_Transmit(hi2c, devAddress, allLedArray[i], CLEAR_DATA);
}

/**
 * @brief  This function is for enable or disable low power mode
 * @param  hi2c I2Cn &handle
 * @param  devAddress Developer address of the PWM LED controller
 * @param  state Low power mode flag, to wake up controller set flag to logic 0
 * @retval None
 */
void PWM_Sleep_State(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t state)
{

	if (state == LOW_POWER_MODE)
	{
		setData = USER_SET_BIT(CLEAR_DATA, MODE1_LED_SLEEP);
		PWM_Data_Transmit(hi2c, devAddress, MODE1_LED_REG, setData);
	}
	else if (state == NORMAL_MODE)
	{
		PWM_Data_Transmit(hi2c, devAddress, MODE1_LED_REG, CLEAR_DATA);

		setData = USER_SET_BIT(USER_SET_BIT(CLEAR_DATA, MODE1_LED_RESTART), MODE1_LED_ALLCALL );
		PWM_Data_Transmit(hi2c, devAddress, MODE1_LED_REG, setData);
	}
}

/**
 * @brief  This function sets Duty Cycle and Delay time for all LED registers
 * @param  hi2c I2Cn &handle
 * @param  devAddress Developer address of the PWM LED controller
 * @param  ledOnTime Time of the delay: 0 - 100 %
 * @param  ledOffTime Duty cycle: 0 - 100 %
 * @retval None
 */
void PWM_Register_All_Set(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint16_t ledOnTime, uint16_t ledOffTime)
{
	pwm_calc pwm = PWM_Calc_Output(ledOnTime, ledOffTime);
	uint32_t transmitBuf[LED_REG_NUMBER] = {(pwm.LEDn_ON & 0xFF),(pwm.LEDn_ON >> 8),(pwm.LEDn_OFF & 0xFF),(pwm.LEDn_OFF >> 8)}; // One byte mask and for ALL_LED_ON_L/ALL_LED_OFF_L

	for(uint8_t i = 0; i < LED_REG_NUMBER; i++)
		PWM_Data_Transmit(hi2c, devAddress, allLedArray[i], transmitBuf[i]);
}

/**
 * @brief  This function sets Duty Cycle and Delay time for specific LED register
 * @param  hi2c I2Cn &handle
 * @param  devAddress Developer address of the PWM LED controller
 * @param  ledOnTime Time of the delay: 0 - 100 %
 * @param  ledOffTime Duty cycle: 0 - 100 %
 * @param  ledChannel Number of the PWM LED channel
 * @retval None
 */
void PWM_Register_Set(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint16_t ledOnTime, uint16_t ledOffTime, uint8_t ledChannel)
{
	if (ledChannel < NUMBER_LED)
	{
		ch_reg_calc reg = PWM_Set_Channel(ledChannel);
		uint8_t ledArray[LED_REG_NUMBER] =  { reg.LEDn_ON_L, reg.LEDn_ON_H, reg.LEDn_OFF_L, reg.LEDn_OFF_H };

		pwm_calc pwm = PWM_Calc_Output(ledOnTime, ledOffTime);
		uint32_t transmitBuf[LED_REG_NUMBER] = {(pwm.LEDn_ON & 0xFF),(pwm.LEDn_ON >> 8),(pwm.LEDn_OFF & 0xFF),(pwm.LEDn_OFF >> 8)};

		for(uint8_t i = 0; i < LED_REG_NUMBER; i++)
			PWM_Data_Transmit(hi2c, devAddress, ledArray[i], transmitBuf[i]);
	}
	else
		return;
}

/**
 * @brief  This function disables specific PWM LED register
 * @param  hi2c I2Cn &handle
 * @param  devAddress Developer address of the PWM LED controller
 * @param  ledChannel Number of the PWM LED channel
 * @retval None
 */
void PWM_Register_Off(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t ledChannel)
{
	if (ledChannel < NUMBER_LED)
	{
		ch_reg_calc reg = PWM_Set_Channel(ledChannel);
		uint8_t ledArray[LED_REG_NUMBER] =  { reg.LEDn_ON_L, reg.LEDn_ON_H, reg.LEDn_OFF_L, reg.LEDn_OFF_H };

		for(uint8_t i = 0; i < LED_REG_NUMBER; i++)
			PWM_Data_Transmit(hi2c, devAddress, ledArray[i], CLEAR_DATA);
	}
	else
		return;
}

/**
 * @brief This function sets PWM frequency of the LEDs
 * The maximum PWM frequency is 1526 Hz if the PRE_SCALE register is set "0x03h"
 * The minimum PWM frequency is 24 Hz if the PRE_SCALE register is set "0xFFh"
 * The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1
 * Internal oscillator clock frequency - 25MHz
 * @param  hi2c I2Cn &handle
 * @param  devAddress Developer address of the PWM LED controller
 * @param  pwmFrequency Set frequency of the PWM LED controller: 24Hz - 1526Hz
 * @retval None
 */
void PWM_Frequency_Set(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint16_t frequency)
{
	static uint32_t PRE_SCALE;

	/*  Calculate frequency prescaler */
	if (frequency < FREQUENCY_MIN)
		PRE_SCALE = PRE_SCALE_MIN; // 24Hz
	else if (frequency > FREQUENCY_MAX)
		PRE_SCALE = PRE_SCALE_MAX; // 1526Hz
	else
		PRE_SCALE = PWM_FREQUENCY(frequency);

/**
 *	If the PCA9685 is operating and then sleep mode is enabled,
 *	RESTART bit (MODE1 bit 7) will be set to logic 1 at the end of the PWM refresh cycle.
 *	The contents of each PWM register are held valid when the clock is off. To restart PWM need to clear bit 4 (SLEEP).
 *	Allow time for oscillator to stabilize and write logic 1 to bit 7 of MODE1 register.
 */
	setData = USER_SET_BIT(CLEAR_DATA, MODE1_LED_SLEEP);
	PWM_Data_Transmit(hi2c, devAddress, MODE1_LED_REG, setData);

	PWM_Data_Transmit(hi2c, devAddress, PRE_SCALE_REG, PRE_SCALE);

	PWM_Data_Transmit(hi2c, devAddress, MODE1_LED_REG, CLEAR_DATA);

	setData = USER_SET_BIT(USER_SET_BIT(CLEAR_DATA, MODE1_LED_RESTART), MODE1_LED_ALLCALL );
	PWM_Data_Transmit(hi2c, devAddress, MODE1_LED_REG, setData);
}



