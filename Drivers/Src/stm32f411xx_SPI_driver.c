/*
 * stm32f411xx_SPI_driver.c
 *
 *  Created on: Mar 13, 2024
 *      Author: vieth
 */

#include <stm32f411xx_SPI_driver.h>

/*
 *	Helper Function prototype
 */
static void SPI_TXE_isr_handler(SPI_Handle_t *pSPIhandle);
static void SPI_RXNE_isr_handler(SPI_Handle_t *pSPIhandle);
static void SPI_OVR_isr_handler();

void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t state){
	if (state == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else {
			/*do nothing*/
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {
//	uint32_t temp_reg = 0;

	/* 0. Enable SPI peripheral clock */
	SPI_PeriClockCtrl(pSPIHandle->SPIx, ENABLE);

	/* 1. Configuration the device mode */
	pSPIHandle->SPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);

	/* 2. Bus config */
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FDUPLEX) {

		pSPIHandle->SPIx->CR1 &= ~(1 << 15);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HDUPLEX) {

		pSPIHandle->SPIx->CR1 |= (1 << 15);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RX) {

		pSPIHandle->SPIx->CR1 &= ~(1 << 15);
		pSPIHandle->SPIx->CR1 |= (1 << 10);
	}

	/* 3. SPI clock speed */
	pSPIHandle->SPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_ClkSpeed << 3);

	/* 4. Data frame format */
	pSPIHandle->SPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_DFF << 11);

	/* 5. config CPOL */
	pSPIHandle->SPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);

	/* 6. config CPHA */
	pSPIHandle->SPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPHA << 0);

	/* 7. config SSM  */
	pSPIHandle->SPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SSM << 9);

	/* 8. pulled SSI to 1  */
	pSPIHandle->SPIx->CR1 |= (1 << 8);

	//for dummy
//	temp_reg |= 1 << 8;

//	pSPIHandle->SPIx->CR2 &= ~(1 << 2 );

//	pSPIHandle->SPIx->CR1 = temp_reg;

}
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Transmit and Receive
 */
uint8_t SPI_Transmit(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
	while (len > 0) {
		/* 1. wait for Tx buffer is empty */
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_CLEAR);

		/* 2. Check DFF */
		if ( pSPIx->CR1 & (1 << SPI_CR1_DFF) ) {
			/* 16 bit frame */
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		} else {
			/* 8 bit frame */
			*((volatile uint8_t *)&pSPIx->DR) = *pTxBuffer;

//			uint8_t dummy_rx = *(volatile uint8_t *)&pSPIx->DR;
			len--;
			pTxBuffer++;
		}
//		pSPIx->CR1
		while ( pSPIx->SR & ( 1 << SPI_SR_BSY ) ); // Wait until the transmission is complete
	}
}

uint8_t SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
	while (len > 0) {
			/* 0. Send dummy byte 0xAA */
//			pSPIx->DR = 0xAA;
			/* 1. wait for Rx buffer has data (not empty) */
			while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_CLEAR);

			/* 2. Check DFF */
			if ( pSPIx->CR1 & (1 << SPI_CR1_DFF) ) {
				/* 16 bit frame */
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				len--;
				len--;
				(uint16_t*)pRxBuffer++;
			} else {
				/* send dummy byte */
//				*(volatile uint8_t *)&pSPIx->DR = 0xAA;
				pSPIx->DR = 0xAA;
				/* 8 bit frame */
				*pRxBuffer = *((volatile uint8_t *)&pSPIx->DR);
				len--;
				pRxBuffer++;
			}
		}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag) {
	if( (pSPIx->SR & Flag)) {
		return FLAG_SET;
	} else {
		return FLAG_CLEAR;
	}
}

uint8_t SPI_Transmit_IT(SPI_Handle_t *pSPIhandle, uint8_t *pTxBuffer, uint32_t len) {
	/*check SPI data register is not in  another Tx transmit */
	if (pSPIhandle->TxState != SPI_TX_BUSY) {
		/* 1.save Tx buffer and Length in to global variables*/
		pSPIhandle->pTxBuffer = pTxBuffer;
		pSPIhandle->TxLength = len;

		/* 1.Mark SPI in busy state */
		pSPIhandle->TxState = SPI_TX_BUSY;

		/* 1.Enable TXEIE control bit to get interrupt whenever TXE flag is set on SR */
		pSPIhandle->SPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		/* 1.Data transmission will be handled by the SPI_ISR code */
	}
	return pSPIhandle->TxState;
}


uint8_t SPI_Receive_IT(SPI_Handle_t *pSPIhandle, uint8_t *pRxBuffer, uint32_t len) {
	/*check SPI data register is not in  another Rx transmit */
	if (pSPIhandle->RxState != SPI_RX_BUSY) {
		/* 1.save Rx buffer and Length in to global variables*/
		pSPIhandle->pRxBuffer = pRxBuffer;
		pSPIhandle->RxLength = len;

		/* 1.Mark SPI in busy state */
		pSPIhandle->RxState = SPI_RX_BUSY;

		/* 1.Enable TXEIE control bit to get interrupt whenever TXE flag is set on SR */
		pSPIhandle->SPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		/* 1.Data transmission will be handled by the SPI_ISR code */
	}
	return pSPIhandle->RxState;
}

void SPI_ISR_Handler(SPI_Handle_t *pSPIhandle) {
	uint8_t flag_check = 0;
	uint8_t isr_flag_check = 0;

	/* 1. Check ISR called by Tx*/
	flag_check = (uint8_t)SPI_GetFlagStatus(pSPIhandle->SPIx, SPI_TXE_FLAG);
	isr_flag_check = pSPIhandle->SPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (flag_check && isr_flag_check) {
		SPI_TXE_isr_handler(pSPIhandle);
	}

	/* 2. Check ISR called by Rx*/
	flag_check = (uint8_t)SPI_GetFlagStatus(pSPIhandle->SPIx, SPI_RXNE_FLAG);
	isr_flag_check = pSPIhandle->SPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (flag_check && isr_flag_check) {
		SPI_RXNE_isr_handler(pSPIhandle);
	}

	/* 3. Check ISR called by overRun*/
	flag_check = (uint8_t)SPI_GetFlagStatus(pSPIhandle->SPIx, SPI_OVR_FLAG);
	isr_flag_check = pSPIhandle->SPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (flag_check && isr_flag_check) {
			SPI_OVR_isr_handler(pSPIhandle);
		}
}

/*
 *	Helper Function
 */
static void SPI_TXE_isr_handler(SPI_Handle_t *pSPIhandle) {
	/* Tranfer data from TxBuffer to SPI_DR */
	if ( pSPIhandle->SPIx->CR1 & (1 << SPI_CR1_DFF) ) {
				/* 16 bit frame */
		pSPIhandle->SPIx->DR = *((uint16_t*)pSPIhandle->pTxBuffer);
		pSPIhandle->TxLength--;
		pSPIhandle->TxLength--;
		(pSPIhandle->pTxBuffer)++;
		(pSPIhandle->pTxBuffer)++;
	} else {
		/* 8 bit frame */
		pSPIhandle->SPIx->DR = *(pSPIhandle->pTxBuffer);
		pSPIhandle->TxLength--;
		pSPIhandle->pTxBuffer++;
	}

	/*Transmit done*/
	if ( !pSPIhandle->TxLength ) {
		/* 1. Prevent interrupt by clear TXEIE bit */
		pSPIhandle->SPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPIhandle->pTxBuffer = NULL;
		pSPIhandle->TxLength = 0;
		pSPIhandle->TxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPIhandle, SPI_EVENT_TX_CMPLT);
	}
}

static void SPI_RXNE_isr_handler(SPI_Handle_t *pSPIhandle) {
	/* 2. Check DFF */
	if ( pSPIhandle->SPIx->CR1 & (1 << SPI_CR1_DFF) ) {
		/* 16 bit frame */
		*((uint16_t*)pSPIhandle->pRxBuffer) = pSPIhandle->SPIx->DR;
		pSPIhandle->RxLength--;
		pSPIhandle->RxLength--;
		pSPIhandle->pRxBuffer++;
		pSPIhandle->pRxBuffer++;
	} else {
		/* 8 bit frame */
		/*dummy byte 0xAA send*/
		pSPIhandle->SPIx->DR = 0xAA;
		/*collect data*/
		*(pSPIhandle->pRxBuffer) = pSPIhandle->SPIx->DR;
		pSPIhandle->RxLength--;
		pSPIhandle->pRxBuffer++;
	}

	/*transmission done, RxBuffer is empty*/
	if (!pSPIhandle->RxLength) {
		pSPIhandle->SPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
		pSPIhandle->pRxBuffer = NULL;
		pSPIhandle->RxLength = 0;
		pSPIhandle->RxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPIhandle, SPI_EVENT_RX_CMPLT);
	}
}
static void SPI_OVR_isr_handler(SPI_Handle_t *pSPIhandle) {
	/* dummy variable to read from DR and SR*/
	uint8_t temp = 0;
	/* 1. Clear the overflag*/
	/* Check SPI is NOT in TX transmit */
	if (pSPIhandle->TxState != SPI_TX_BUSY) {
		temp = pSPIhandle->SPIx->DR;
		temp = pSPIhandle->SPIx->SR;
	}
	(void) temp;
	SPI_ApplicationEventCallback(pSPIhandle, SPI_EVENT_OVR_ERR);

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t state) {
	if (state == ENABLE) {
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIhandle, uint8_t event) {

}

