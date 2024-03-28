/*
 * spi.c
 *
 *  Created on: Mar 28, 2024
 *      Author: HAU5HC
 */

/*
 * SPI1_SS		-	A4
 * SPI1_CLK 	- 	A5
 * SPI1_MISO 	-	A6
 * SPI1_MOSI	-	A7
 *
 * SPI2_CLK		-	B10
 * SPI2_MISO	-	B14
 * SPI2_MOSI	-	B15
 * SPI2_SS		-	B9
 *
 */

#include "stm32f411xx.h"
#include "stm32f411xx_GPIO_driver.h"
#include "stm32f411xx_SPI_driver.h"

void SPI1_Init(void) {
	/* 1. Configure GPIO for SPI1 */
	GPIO_Handle_t GPIO_SPI1;
	memset(GPIO_SPI1, 0, sizeof(GPIO_SPI1));

	GPIO_SPI1->pGPIO = GPIOA;
	GPIO_SPI1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_SPI1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_SPI1.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PU_PD;
	GPIO_SPI1.GPIO_PinConfig.GPIO_PinOutputType = GPIO_OUT_PUSHPULL;
	GPIO_SPI1.GPIO_PinConfig.GPIO_PinAltFuncMode = GPIO_ALTFUNC_5;

	/* SPI1_SS */
	GPIO_SPI1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&GPIO_SPI1);

	/* SPI1_CLK */
	GPIO_SPI1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&GPIO_SPI1);

	/* SPI1_MISO */
	GPIO_SPI1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&GPIO_SPI1);

	/* SPI1_MOSI */
	GPIO_SPI1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&GPIO_SPI1);

	/* 2. Configure SPI1 properties */
	SPI_Handle_t SPI1_Handle;
	memset(SPI1_Handle, 0, sizeof(SPI1_Handle));

	SPI1_Handle->SPIx = SPI1;
	SPI1_Handle
}


