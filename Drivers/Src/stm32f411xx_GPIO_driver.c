/*
 * stm32f411xx_GPIO_driver.c
 *
 *  Created on: Mar 10, 2024
 *      Author: vieth
 */

#include <stm32f411xx_GPIO_driver.h>

/*
 * APIs supported by this driver
 */

void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t state){
	if (state == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}else {
			/*do nothing*/
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandler) {
	uint32_t temp = 0; /*temporary register*/
	uint8_t reg_selection = 0;
	uint8_t pin_selection = 0;
	uint8_t port_order = 0;

	/* 1. configuration gpio pin mode */
	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = pGPIOHandler->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandler->pGPIO->MODER &= ~(0x03 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandler->pGPIO->MODER |= temp;

	} else {
		/*interrupt mode*/
		if ( pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			/* Configuration for Falling Edge IT */
			/* 1. Enable EXTI_FTSR */
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			/* 2. Clear the corresponding RTSR */
			EXTI->RTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT ) {
			/* Configuration for Rising Edge IT*/
			/* 1. Enable EXTI_RTSR */
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			/* 2. Clear the corresponding FTSR */
			EXTI->FTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT ) {
			/* Configuration for Rising and Falling Edge IT*/
			/* 1. Enable EXTI_RTSR */
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
			/* 2. Enable EXTI_FTSR */
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
		}

		/* 2. Configure the GPIO port Selection in SYSCFG_EXTICR */
		reg_selection = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber / 4;
		port_order = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber % 4;
		SYSCFG_PCLK_EN();
		/* clear bit */
		SYSCFG->EXTICRx[reg_selection] &= ~( 0xF << (port_order*4) );
		/* set bit */
		SYSCFG->EXTICRx[reg_selection] |= ( (GPIO_BASEADDR_2_CODE(pGPIOHandler->pGPIO)) << (port_order*4) );

		/* 3. Enable EXTI interrupt delivery using IMR by un-masking bit
		 * *only un-mask the corresponding interrupt line
		 */
		EXTI->IMR |= 1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber;

	}
	temp = 0;

	/* 2. configuration gpio pin speed */
	temp = pGPIOHandler->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIO->OSPEEDR &= ~(0x03 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIO->OSPEEDR |= temp;
	temp = 0;

	/* 3. configuration gpio pin pu/pd */
	temp = pGPIOHandler->GPIO_PinConfig.GPIO_PinPuPdCtrl << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIO->PUPDR &= ~(0x03 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIO->PUPDR |= temp;
	temp = 0;

	/* 4. configuration gpio pin output type */
	temp = pGPIOHandler->GPIO_PinConfig.GPIO_PinOutputType << (pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIO->OTYPER &= ~(0x01 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIO->OTYPER |= temp;
	temp = 0;

	/* 5. configuration gpio pin alternate function */
	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		reg_selection = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber / 8; /*total pin = 16*/
		pin_selection = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = pGPIOHandler->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * pin_selection);
		pGPIOHandler->pGPIO->AFR[reg_selection] &= ~(0b1111 << (4 * pin_selection));
		pGPIOHandler->pGPIO->AFR[reg_selection] |= temp;
	}
	temp = 0;

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
		if (pGPIOx == GPIOA) {
			GPIOA_REG_RESET();
		} else if (pGPIOx == GPIOB) {
			GPIOB_REG_RESET();
		} else if (pGPIOx == GPIOC) {
			GPIOC_REG_RESET();
		} else if (pGPIOx == GPIOD) {
			GPIOD_REG_RESET();
		} else if (pGPIOx == GPIOE) {
			GPIOE_REG_RESET();
		}else {
			/*do nothing*/
		}

}

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value=0;
	value = (uint8_t) (pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value=0;
	value = (uint16_t) (pGPIOx->IDR);
	return value;
}

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value) {
	if (value == GPIO_PIN_SET) {
		/*set pin*/
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		/*reset pin*/
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	/*write 16bit data to GPIO port*/
	pGPIOx->ODR |= (value);

}
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= ( 1 << PinNumber);
}


void GPIO_IRQ_ISR_Config(uint8_t IRQNumber, uint8_t state) {
	if ( state == ENABLE) {
		/* Enable interrupt */
		if ( IRQNumber <= 31 ) {
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		} else if ( IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		} else if ( IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	} else {
		/* Disable interrupt*/
		if ( IRQNumber <= 31 ) {
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		} else if ( IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		} else if ( IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

void GPIO_IRQ_PRIO_Config(uint8_t IRQNumber, uint8_t IRQPriority) {
	uint8_t reg_index = 0; /* 0 - 59 */
	uint8_t reg_order = 0; /* 0 - 3 */

	reg_index = IRQNumber / 4;
	reg_order = IRQNumber % 4;
	/* clear bit first */
	*( NVIC_IPR_BASEADDR + reg_index ) &= ~( 0xF << (reg_order*8 + 4) );
	/* then set bit */
	*( NVIC_IPR_BASEADDR + reg_index ) |= ( IRQPriority << (reg_order*8 + 4) );
}

void GPIO_IRQHandling(uint8_t PinNumber) {
	/* Check and clear the EXTI_PR (Pending Register) */
	if (EXTI->PR & (1 << PinNumber)) {
		/* Clear by program to 1 */
		EXTI->PR |= (1 << PinNumber);
	}
}
