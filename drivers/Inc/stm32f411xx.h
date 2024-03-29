/*
 * stm32f411xx.h
 *
 *  Created on: Dec 12, 2023
 *      Author: HAU5HC
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include<stdint.h>
#include<stddef.h>
#include<stdio.h>

#define __vo volatile
//#define __weak __atribute__((weak))

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
#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800)
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00)

/*
 * Base address of all peripherals hanging on AHB1 bus
 */
#define GPIOA_BASEADDR		(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1_BASEADDR + 0x1000)

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
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)

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

/*Clock Enable for SPIx*/
#define SPI1_PCLK_EN()	(RCC->APB1ENR |= 1 << 14)
#define SPI2_PCLK_EN()	(RCC->APB2ENR |= 1 << 12)

/*
 *	Bit Position of SPI_CR1
 */
#define SPI_CR1_RXNE		0
#define SPI_CR1_TXE		1
#define SPI_CR1_CHSIDE	2
#define SPI_CR1_UDR		3
#define SPI_CR1_CRCERR	4
#define SPI_CR1_MODF		5
#define SPI_CR1_OVR		6
#define SPI_CR1_BSY		7
#define SPI_CR1_DFF		11

/*
 *	Bit Position of SPI_CR2
 */
#define SPI_CR2_RXNE		0
#define SPI_CR2_TXE		1
#define SPI_CR2_CHSIDE	2
#define SPI_CR2_UDR		3
#define SPI_CR2_CRCERR	4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_DFF		11

/*
 *	Bit Position of SPI_SR
 */
#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define SPI_SR_FRE		8

/********************************************************************/
/* EXTI */
/********************************************************************/

/*
 * peripheral Register Definition structure for EXTI
 */
typedef struct {
	__vo uint32_t IMR;		/* Interrupt mask register */				/*offset = 0x00*/
	__vo uint32_t EMR;		/* Event mask register */					/*offset = 0x04*/
	__vo uint32_t RTSR;		/* Rising trigger selection register */		/*offset = 0x08*/
	__vo uint32_t FTSR;		/* Falling trigger selection register */	/*offset = 0x0C*/
	__vo uint32_t SWIER;	/* Software interrupt event register */		/*offset = 0x10*/
	__vo uint32_t PR;		/* Pending register */						/*offset = 0x14*/
} EXTI_RegDef_t;

#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)


/********************************************************************/
/* End of EXTI */
/********************************************************************/

/********************************************************************/
/* SYSCFG */
/********************************************************************/

/*
 * peripheral Register Definition structure for EXTI
 */
typedef struct {
	__vo uint32_t MEMRMP;		/* memory re-map register */						/*offset = 0x00*/
	__vo uint32_t PMC;			/* peripheral mode configuration register */		/*offset = 0x04*/
	__vo uint32_t EXTICRx[4];	/* external interrupt configuration register 1:4 */	/*offset = 0x08 - 0x14*/
	uint32_t	RESERVED;															/*offset = 0x18*/
	__vo uint32_t CMPCR;		/* cell control register */							/*offset = 0x20*/
} SYSCFG_RegDef_t;

#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/********************************************************************/
/* End of SYSCFG */
/********************************************************************/

/*
 * Clock Enable for GPIOx
 */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))	/* GPIOA Peripheral clock Enable*/
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))	/* GPIOB Peripheral clock Enable*/
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))	/* GPIOC Peripheral clock Enable*/
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))	/* GPIOD Peripheral clock Enable*/
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))	/* GPIOE Peripheral clock Enable*/

/*
 * Reset GPIOx
 */
#define GPIOA_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)	/* GPIOA Peripheral register reset*/
#define GPIOB_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)	/* GPIOB Peripheral register reset*/
#define GPIOC_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)	/* GPIOC Peripheral register reset*/
#define GPIOD_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)	/* GPIOD Peripheral register reset*/
#define GPIOE_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)	/* GPIOE Peripheral register reset*/

/*
 * Clock Disable for GPIOx
 */
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))	/* GPIOA Peripheral clock Disable*/

/*
 * General macro
 */
#define ENABLE		1
#define DISABLE		0
#define GPIO_PIN_SET		1
#define GPIO_PIN_RESET		0
#define FLAG_SET			1
#define FLAG_CLEAR			0

#endif /* INC_STM32F411XX_H_ */
