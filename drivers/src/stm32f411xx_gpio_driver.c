/*
 * stm32f411xx_GPIO_driver.c
 *
 *  Created on: Mar 10, 2024
 *      Author: vieth
 */

#include<stm32f411xx_GPIO_driver.h>

/*
 * APIs supported by this driver
 */

void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t state);

void GPIO_Init(GPIO_Handle_t *pGPIOHandler);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);


void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void GPIO_IRQHandling(uint8_t PinNumber);
