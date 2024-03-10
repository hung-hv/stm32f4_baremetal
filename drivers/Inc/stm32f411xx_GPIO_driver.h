/*
 * stm32f411xx_GPIO_driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: vieth
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include<stm32f411xx.h>

/*
 * Configuration for GPIO Pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdCtrl;
	uint8_t GPIO_PinOutputType;
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConf_t;

/*
 * Handle structure for GPIO pin
 */
typedef struct {
	GPIO_RegDef_t* pGPIO;
	GPIO_PinConf_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * APIs supported by this driver
 */

void GPIO_Init(void);
void GPIO_DeInit(void);
void GPIO_PeriClockCtrl(void);
void GPIO_ReadInputPin(void);
void GPIO_ReadInputPort(void);
void GPIO_WriteOutputPin(void);
void GPIO_WriteOutputPort(void);
void GPIO_TogglePin(void);
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
