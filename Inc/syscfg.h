/*
 * syscfg.h
 *
 *  Created on: 14 Oct 2022
 *      Author: Veli Ertunç
 */

#ifndef SYSCFG_H_
#define SYSCFG_H_
#include "baseaddr.h"

typedef volatile struct
{
	uint32_t MEMRMP;       /* Remap register,                   			Address offset: 0x00      */
	uint32_t PMC;          /* Peripheral mode configuration register,     	Address offset: 0x04      */
	uint32_t EXTICR[4];    /* External interrupt configuration registers,	Address offset: 0x08-0x14 */
	uint32_t RESERVED1[2]; /* Reserved, 									Address offset: 0x18-0x1C */
	uint32_t CMPCR;        /* Compensation cell control register,         	Address offset: 0x20      */
	uint32_t RESERVED2[2]; /* Reserved, 									Address offset: 0x24-0x28 */
}SYSCFG_RegDef_t;


#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASE)

void SYSCFG_SetPeripheralClock(uint8_t enabled);

#endif /* SYSCFG_H_ */
