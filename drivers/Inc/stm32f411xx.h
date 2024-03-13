/*
 * stm32f411xx.h
 *
 *  Created on: Dec 12, 2023
 *      Author: HAU5HC
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include<stdint.h>

#define __vo volatile

/* system memories*/
#define MCU_BASEADDR		0x08000000
#define SRAM_BASEADDR		MCU_BASEADDR
//#define

/* System bus define */
//#define
#define APB1_BASEADDR		0x40000000
#define APB2_BASEADDR		0x40010000
#define AHB1_BASEADDR		0x40020000
#define AHB2_BASEADDR		0x50000000

/*
 * Base address of all peripherals hanging on APB1 bus
 */
#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00)

/*
 * Base address of all peripherals hanging on APB2 bus
 */
#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR		(APB2_BASEADDR + 0x3400)

/*
 * Base address of all peripherals hanging on AHB1 bus
 */
#define GPIOA_BASEADDR		(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x0800)

#define RCC_BASEADDR		(AHB1_BASEADDR + 0x3800)

/*
 * Base address of all peripherals hanging on APB2 bus
 */


/*
 * peripheral register definition structure for GPIO
 */
typedef struct {
	__vo uint32_t MODER;			/*offset = 0x00*/
	__vo uint32_t OTYPER;		/*offset = 0x04*/
	__vo uint32_t OSPEEDR;		/*offset = 0x08*/
	__vo uint32_t PUPDR;			/*offset = 0x0C*/
	__vo uint32_t IDR;			/*offset = 0x10*/
	__vo uint32_t ODR;			/*offset = 0x14*/
	__vo uint32_t BSRR;			/*offset = 0x1C*/
	__vo uint32_t LCKR;			/*offset = 0x20*/
	__vo uint32_t AFR[2];		/*offset = 0x00*/
} GPIO_RegDef_t;

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)

/*
 * peripheral register definition structure for RCC
 */
typedef struct {
	__vo uint32_t CR;			/*offset = 0x00*/
	__vo uint32_t PLLCFGR;		/*offset = 0x04*/
	__vo uint32_t CFGR;		/*offset = 0x08*/
	__vo uint32_t CIR;			/*offset = 0x0C*/
	__vo uint32_t AHB1RSTR;			/*offset = 0x10*/
	__vo uint32_t AHB2RSTR;			/*offset = 0x14*/
	uint32_t 	RESERVED0[2];		/*offset = 0x18 - 0x0C*/
	__vo uint32_t APB1RSTR;			/*offset = 0x1C*/
	__vo uint32_t APB2RSTR;			/*offset = 0x20*/
	uint32_t 	RESERVED1[2];		/*offset = 0x28 - 0x2C*/
	__vo uint32_t AHB1ENR;		/*offset = 0x00*/
	__vo uint32_t AHB2ENR;		/*offset = 0x00*/
	uint32_t 	RESERVED2[2];		/*offset = 0x38 - 0x3C*/
	__vo uint32_t APB1ENR;		/*offset = 0x00*/
	__vo uint32_t APB2ENR;		/*offset = 0x00*/
	uint32_t 	RESERVED3[2];		/*offset = 0x48 - 0x4C*/
	__vo uint32_t AHB1LPENR;		/*offset = 0x00*/
	__vo uint32_t AHB2LPENR;		/*offset = 0x00*/
	uint32_t 	RESERVED4[2];		/*offset = 0x58 - 0x5C*/
	__vo uint32_t APB1LPENR;		/*offset = 0x00*/
	__vo uint32_t APB2LPENR;		/*offset = 0x00*/
	uint32_t 	RESERVED5[2];		/*offset = 0x68 - 0x6C*/
	__vo uint32_t BDCR;		/*offset = 0x00*/
	__vo uint32_t CSR;		/*offset = 0x00*/
	uint32_t 	RESERVED6[2];		/*offset = 0x78 - 0x7C*/
	__vo uint32_t SSCGR;		/*offset = 0x00*/
	__vo uint32_t PLLI2SCFGR;		/*offset = 0x00*/
	uint32_t 	RESERVED7;		/*offset = 0x88*/
	__vo uint32_t DCKCFGR;		/*offset = 0x8C*/

} RCC_RegDef_t;

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)



/*
 * peripheral register definition structure for SPI
 */

typedef struct {
	__vo uint32_t CR1;		/* Control 1*/			/*offset = 0x00*/
	__vo uint32_t CR2;		/* Control 2*/			/*offset = 0x04*/
	__vo uint32_t SR;		/* Status */			/*offset = 0x08*/
	__vo uint32_t DR;		/* Data */				/*offset = 0x0C*/
	__vo uint32_t CRCPR;	/* CRC polynomial */	/*offset = 0x10*/
	__vo uint32_t RXCRCR;	/* RX CRC */			/*offset = 0x14*/
	__vo uint32_t TXCRCR;	/* TX CRC */			/*offset = 0x10*/
	__vo uint32_t I2SCFGR;	/* I2S config */		/*offset = 0x14*/
	__vo uint32_t I2SPR;	/* I2S prescale */		/*offset = 0x14*/
} SPI_RegDef_t;

#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)

/*Clock enable macro for SPIx*/
#define SPI1_PCLK_EN()	(RCC->APB1ENR |= 1 << 14)
#define SPI2_PCLK_EN()	(RCC->APB2ENR |= 1 << 12)

/*
 * Clock Enable for GPIOx
 */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))	/* GPIOA Peripheral clock Enable*/

/*
 * Clock Disable for GPIOx
 */
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))	/* GPIOA Peripheral clock Disable*/

#endif /* INC_STM32F411XX_H_ */
