/*
 * syscfg.h
 *
 *  Created on: 14 Oct 2022
 *      Author: Veli Ertunç
 */

#ifndef SYSCFG_H_
#define SYSCFG_H_
#include "types.h"
#include "baseaddr.h"

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASE)

void SYSCFG_SetPeripheralClock(uint8_t enabled)
{
	if (enabled) {
		RCC->APB2ENR |= (1 << 14);
	} else {
		RCC->APB2ENR &= ~(1 << 14);
	}
}

#endif /* SYSCFG_H_ */
