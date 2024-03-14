/*
 * stm32f411xx_SPI_deiver.h
 *
 *  Created on: Mar 13, 2024
 *      Author: vieth
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_
/*
 * Configuration structure for SPIx peripheral
 */

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_ClkSpeed;
	uint8_t DFF;				/* data frame format */
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
} SPI_Handle_t;

/*.
 * 		API for this driver
 */


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
