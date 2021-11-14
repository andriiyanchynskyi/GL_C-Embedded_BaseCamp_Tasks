/*
 * pca9685_pwm.h
 *
 *  Created on: 12 лист. 2021 р.
 *      Author: rozen
 */

#ifndef INC_PCA9685_PWM_H_
#define INC_PCA9685_PWM_H_

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "stm32f4xx_hal.h"

/* Defines  ------------------------------------------------------------------*/
#define MODE1_LED_REG		0u
#define MODE2_LED_REG		1u

#define ALL_LED_ON_L		250u
#define ALL_LED_ON_H		251u
#define ALL_LED_OFF_L		252u
#define ALL_LED_OFF_H		253u

#define NUMBER_LED 			16u

#define PRE_SCALE_REG		254u

#define MODE1_LED_RESTART	(1u << 7)
#define MODE1_LED_EXTCLK	(1u << 6)
#define MODE1_LED_AI		(1u << 5)
#define MODE1_LED_SLEEP		(1u << 4)
#define MODE1_LED_SUB1		(1u << 3)
#define MODE1_LED_SUB2		(1u << 2)
#define MODE1_LED_SUB3		(1u << 1)
#define MODE1_LED_ALLCALL	(1u << 0)

#define MODE2_LED_INVRT		(1u << 4)
#define MODE2_LED_OCH		(1u << 3)
#define MODE2_LED_OUTDRV	(1u << 2)
#define MODE2_LED_OUTNE1	(1u << 1)
#define MODE2_LED_OUTNE0	(1u << 0)

/* Exported functions prototypes ---------------------------------------------*/
void PWM_LED_Init(I2C_HandleTypeDef *hi2c, uint8_t devId);
void LED_Register_All_Set(I2C_HandleTypeDef *hi2c, uint8_t devId, uint16_t delayTime, uint16_t dutyCycle);
void LED_Register_All_Off(I2C_HandleTypeDef *hi2c, uint8_t devId);
void LED_Sleep_State(I2C_HandleTypeDef *hi2c, uint8_t devId, uint8_t state);
void PWM_Frequency_Set(I2C_HandleTypeDef *hi2c, uint8_t devId, uint16_t frequency);

#endif /* INC_PCA9685_PWM_H_ */
