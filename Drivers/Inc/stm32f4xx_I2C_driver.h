/*
 * stm32f4xx_I2C_driver.h
 *
 *  Created on: Mar 18, 2024
 *      Author: vieth
 */

#ifndef INC_STM32F4XX_I2C_DRIVER_H_
#define INC_STM32F4XX_I2C_DRIVER_H_

#include <stm32f411xx.h>

/*
 * Struct config I2C
 */
typedef struct {
	uint32_t I2C_SCLspeed;
	uint8_t	I2C_Address;
	uint8_t	I2C_ACKcontrol;
	uint16_t I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct {

};

#endif /* INC_STM32F4XX_I2C_DRIVER_H_ */
