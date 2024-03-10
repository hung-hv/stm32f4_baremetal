/*
 * stm32f411xx.h
 *
 *  Created on: Dec 12, 2023
 *      Author: HAU5HC
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

/* system memories*/
#define MCU_BASEADDR		0x08000000
#define SRAM_BASEADDR		MCU_BASEADDR
//#define

/* System bus define */
#define
#define APB1_BASEADDR		0x40000000
#define APB2_BASEADDR		0x40010000
#define AHB1_BASEADDR		0x40020000
#define AHB2_BASEADDR		0x50000000

/*
 * Base address of all peripherals hanging on APB1 bus
 */
#define GPIOA_BASEADDR		(APB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(APB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(APB1_BASEADDR + 0x0800)

/*
 * Base address of all peripherals hanging on APB2 bus
 */

#endif /* INC_STM32F411XX_H_ */
