/*
 * nvic.h
 *
 *  Created on: 14 Oct 2022
 *      Author: Veli Ertunç
 */

#ifndef NVIC_H_
#define NVIC_H_
#include <stddef.h>
#include <stdint.h>
#include "baseaddr.h"

/*
 * NVIC ISERx register Addresses
 */

#define NVIC_ISER0     		      	((_vu32*)0xE000E100 )
#define NVIC_ISER1     		      	((_vu32*)0xE000E104 )
#define NVIC_ISER2    		      	((_vu32*)0xE000E108 )
#define NVIC_ISER3    		     	((_vu32*)0xE000E10c )

/*
 * NVIC ICERx register Addresses
 */
#define NVIC_ICER0 					((_vu32*)0XE000E180)
#define NVIC_ICER1					((_vu32*)0XE000E184)
#define NVIC_ICER2  				((_vu32*)0XE000E188)
#define NVIC_ICER3					((_vu32*)0XE000E18C)

/*
 * Number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  	4

/*
 * IRQ Configuration and ISR handling
 */
void NVIC_EnableIRQ(uint8_t irqNo);
void NVIC_DisableIRQ(uint8_t irqNo);
void NVIC_SetPriority(uint8_t irqNo, uint32_t priority);

#endif /* NVIC_H_ */
