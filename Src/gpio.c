/*
 * gpio.c
 *
 *  Created on: Oct 14, 2022
 *      Author: Veli Ertunç
 */
#include "gpio.h"

void GPIO_SetPeripheralClock(GPIO_RegDef_t *pGPIOx, uint8_t enabled) {
	if (enabled) {
		RCC->AHB1ENR |= (1 << GPIO_TO_CODE(pGPIOx));
	} else {
		RCC->AHB1ENR &= ~(1 << GPIO_TO_CODE(pGPIOx));
	}
}

/**Initializes the pin with configuration given in handle
 *	@param pHandle: The handle for pin
 */
void GPIO_Init(GPIO_Handle_t *pHandle) {
	uint32_t temp = 0;
	uint8_t pinNo = pHandle->config.PinNo;
	uint8_t mode = pHandle->config.Mode;
	uint8_t speed = pHandle->config.Speed;
	uint8_t pull = pHandle->config.Pull;
	uint8_t outputType = pHandle->config.OutputType;
	uint8_t afMode = pHandle->config.AltFunMode;

	GPIO_SetPeripheralClock(pHandle->pGPIOx, ENABLE);

	/* 1. Configure the pin mode */
	if (mode < GPIO_MODE_IT_FT) { //Non-interrupt mode
		pHandle->pGPIOx->MODER &= ~(3 << (2 * pinNo)); //Clear the bits
		pHandle->pGPIOx->MODER |= (mode << (2 * pinNo)); //Set pin mode
	} else {
		// Interrupt mode
		// Configure FTSR & RTSR bit positions
		if (mode == GPIO_MODE_IT_FT) {
			EXTI->FTSR |= (1 << pinNo);
			EXTI->RTSR &= ~(1 << pinNo);
		}

		if (mode == GPIO_MODE_IT_RT) {
			EXTI->FTSR &= ~(1 << pinNo);
			EXTI->RTSR |= (1 << pinNo);
		}

		if (mode == GPIO_MODE_IT_RFT) {
			EXTI->FTSR |= (1 << pinNo);
			EXTI->RTSR |= (1 << pinNo);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t portcode = GPIO_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_SetPeripheralClock(ENABLE);
		SYSCFG->EXTICR[pinNo / 4] = portcode << ((pinNo % 4) * 4);

		//3 . Enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1U << pinNo;
	}
	/* 2. Configure the pin speed */
	pHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pinNo));
	pHandle->pGPIOx->OSPEEDR |= (speed << 2 * pinNo);

	/* 3. Configure output type */
	pHandle->pGPIOx->OTYPER &= ~(0x1 << pinNo);
	pHandle->pGPIOx->OTYPER |= (outputType << pinNo);

	/* 4. Configure pull up/down resistor */
	pHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pinNo));
	pHandle->pGPIOx->PUPDR |= (pull << 2 * pinNo);

	if (mode == GPIO_MODE_AF) {
		//configure the alt function registers.
		pGPIOHandle->pGPIOx->AFR[pinNo / 8] &= ~(0xF << (4 * (pinNo % 8))); //clearing
		pGPIOHandle->pGPIOx->AFR[pinNo / 8] |= (afMode << (4 * (pinNo % 8)));
	}
}

void GPIO_Reset(GPIO_RegDef_t *pGPIOx) {
	(RCC->AHB1RSTR |= (1 << GPIO_TO_CODE(pGPIOx)));
	(RCC->AHB1RSTR &= ~(1 << GPIO_TO_CODE(pGPIOx)));
}

uint32_t GPIO_Read(GPIO_RegDef_t *pGPIOx, uint8_t pin) {
	return ((pGPIOx->IDR >> pin) & 0x1U);
}

uint32_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx) {
	return pGPIOx->IDR;
}

void GPIO_Write(GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t value) {
	if (value == HIGH) {
		pGPIOx->ODR |= (1U << pin);
	} else {
		pGPIOx->ODR &= ~(1U << pin);
	}
}

void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint32_t value) {
	pGPIOx->ODR = value;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pin) {
	pGPIOx->ODR ^= (1U << pin);
}

void GPIO_IRQInterruptConfig(uint8_t irqNo, uint8_t enabled) {
	if (enabled) {
		if (irqNo <= 31) {
			//program ISER0 register
			*NVIC_ISER0 |= (1 << irqNo);
		} else if (irqNo > 31 && irqNo < 64) //32 to 63
				{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (irqNo % 32));
		} else if (irqNo >= 64 && irqNo < 96) {
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= (1 << (irqNo % 64));
		}
	} else {
		if (irqNo <= 31) {
			//program ICER0 register
			*NVIC_ICER0 |= (1 << irqNo);
		} else if (irqNo > 31 && irqNo < 64) //32 to 63
				{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (irqNo % 32));
		} else if (irqNo >= 64 && irqNo < 96) {
			//program ICER2 register //64 to 95
			*NVIC_ICER2 |= (1 << (irqNo % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t irqNo, uint32_t priority) {
	uint8_t iprx = irqNo / 4;
	uint8_t iprx_section = irqNo % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE + iprx) |= (priority << shift_amount);
}

void GPIO_IRQHandling(uint8_t pin) {
	//clear the exti pr register corresponding to the pin number
	if (EXTI->PR & (1 << pin)) {
		EXTI->PR |= (1 << pin);
	}
}
