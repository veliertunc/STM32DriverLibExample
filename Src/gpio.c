/*
 * gpio.c
 *
 *  Created on: Oct 14, 2022
 *      Author: Veli Ertunç
 */
#include "gpio.h"

void Gpio_SetPeripheralClock(GPIO_RegDef_t* GPIOx, uint8_t enabled)
{
	if (enabled) {
		RCC->AHB1ENR |= (1 << GPIO_TO_CODE(GPIOx));
	} else {
		RCC->AHB1ENR &= ~(1 << GPIO_TO_CODE(GPIOx));
	}
}

void Gpio_Reset(GPIO_RegDef_t* GPIOx)
{
	(RCC->AHB1RSTR |= (1 << GPIO_TO_CODE(GPIOx)));
	(RCC->AHB1RSTR &= ~(1 << GPIO_TO_CODE(GPIOx)));
}

