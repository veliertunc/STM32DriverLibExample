/*
 * nvic.c
 *
 *  Created on: 15 Oct 2022
 *      Author: Veli Ertunç
 */
#include "nvic.h"

void NVIC_EnableIRQ(uint8_t irqNo) {
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
}

void NVIC_DisableIRQ(uint8_t irqNo) {

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

void NVIC_SetPriority(uint8_t irqNo, uint32_t priority) {
	uint8_t iprx = irqNo / 4;
	uint8_t iprx_section = irqNo % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE + iprx) |= (priority << shift_amount);
}
