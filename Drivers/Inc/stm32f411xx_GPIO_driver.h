/*
 * stm32f411xx_GPIO_driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: vieth
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include <stm32f411xx.h>

/*
 * Configuration for GPIO Pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;		/* @GPIO mode */
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
 * GPIO Pin number
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO mode
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
/*interrupt mode*/
#define GPIO_MODE_IT_RT		4	/* interrupt raising edge trigger */
#define GPIO_MODE_IT_FT		5	/* interrupt falling edge trigger */
#define GPIO_MODE_IT_RFT	6	/* interrupt raising and falling edge trigger */

/*
 * GPIO output type
 */
#define GPIO_OUT_PUSHPULL	0
#define GPIO_OUT_OPENDRAIN	1

/*
 * GPIO speed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * GPIO Pull-up/Pull-down
 */
#define GPIO_NO_PU_PD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * APIs supported by this driver
 */

void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t state);

void GPIO_Init(GPIO_Handle_t *pGPIOHandler);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);


void GPIO_IRQ_ISR_Config(uint8_t IRQNumber, uint8_t state);

void GPIO_IRQ_PRIO_Config(uint8_t IRQNumber, uint8_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
