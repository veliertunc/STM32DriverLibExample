/*
 * syscfg.c
 *
 *  Created on: 15 Oct 2022
 *      Author: Veli Ertunç
 */

#include "syscfg.h"

void SYSCFG_SetPeripheralClock(uint8_t enabled)
{
	if (enabled) {
		RCC->APB2ENR |= (1 << 14);
	} else {
		RCC->APB2ENR &= ~(1 << 14);
	}
}
