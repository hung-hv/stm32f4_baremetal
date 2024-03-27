/*
 * stm32f411xx_SPI_driver.h
 *
 *  Created on: Mar 13, 2024
 *      Author: vieth
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include <stm32f411xx.h>
/*
 * Configuration structure for SPIx peripheral
 */

typedef struct {
	uint8_t SPI_DeviceMode;		/* @SPI_DeviceMode */
	uint8_t SPI_BusConfig;		/* SPI_BusConfig */
	uint8_t SPI_ClkSpeed;
	uint8_t SPI_DFF;				/* data frame format */
	uint8_t SPI_CPOL;			/* */
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;			/* SS pin management */
} SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t *SPIx;		/* pointer to hold address of SPIx [1:4]*/
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLength;
	uint32_t RxLength;
	uint8_t TxState;
	uint8_t RxState;
} SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_MODE_MASTER		0
#define SPI_MODE_SLAVE		1

/*
 * @SPI_BusConfig
 */
#define	SPI_BUS_FDUPLEX			0
#define	SPI_BUS_HDUPLEX			1
#define	SPI_BUS_SIMPLEX_RX		2
//#define	SPI_BUS_SIMPLEX_TX		3

/*
 * SPI_ClkSpeed
 */
#define	SPI_CLOCK_SPEED_DIV_2			0
#define	SPI_CLOCK_SPEED_DIV_4			1
#define	SPI_CLOCK_SPEED_DIV_8			2
#define	SPI_CLOCK_SPEED_DIV_16			3
#define	SPI_CLOCK_SPEED_DIV_32			4
#define	SPI_CLOCK_SPEED_DIV_64			5
#define	SPI_CLOCK_SPEED_DIV_128			6
#define	SPI_CLOCK_SPEED_DIV_256			7

/*
 * @SPI_DFF
 */
#define	SPI_DATA_FRAME_8			0
#define	SPI_DATA_FRAME_16			1

/*
 * @SPI_CPOL
 */
#define	SPI_CPOL_LOW			0
#define	SPI_CPOL_HIGH			1

/*
 * @SPI_CPHA
 */
#define	SPI_CPHA_LOW			0
#define	SPI_CPHA_HIGH			1

/*
 * @SPI_SSM
 */
#define	SPI_SSM_DIS			0
#define	SPI_SSM_EN			1


/*
 * SPI STATUS REGISTER flag
 */
#define	SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define	SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define	SPI_BSY_FLAG		(1 << SPI_SR_BSY)

#define	SPI_OVR_FLAG		(1 << SPI_SR_OVR)
/*
 * SPI STATE status
 */
#define SPI_READY	0
#define SPI_TX_BUSY	1
#define SPI_RX_BUSY	2

/*
 * SPI EVENT status
 */
#define SPI_EVENT_RX_CMPLT	0
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_OVR_ERR	2

/*.
 * 		API for this driver
 */

void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t state);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Transmit and Receive
 */
uint8_t SPI_Transmit(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag);

uint8_t SPI_Transmit_IT(SPI_Handle_t *pSPIhandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_Receive_IT(SPI_Handle_t *pSPIhandle, uint8_t *pRxBuffer, uint32_t len);

void SPI_ISR_Handler(SPI_Handle_t *pSPIhandle);

/*
 *	Application Callback
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIhandle, uint8_t event);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
