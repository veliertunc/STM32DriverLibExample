/*
 * exti.h
 * Header for EXTI peripheral
 *  Created on: 14 Oct 2022
 *      Author: Veli Ertunç
 */

#ifndef EXTI_H_
#define EXTI_H_

#include "baseaddr.h"

/*
 * EXTI register definition
 */
typedef volatile struct
{
	uint32_t IMR;    /* Interrupt mask register ,          	  	    Address offset: 0x00 */
	uint32_t EMR;    /* Event mask register,                		Address offset: 0x04 */
	uint32_t RTSR;   /* Rising trigger selection register,  		Address offset: 0x08 */
	uint32_t FTSR;   /* Falling trigger selection register, 		Address offset: 0x0C */
	uint32_t SWIER;  /* Software interrupt event register,  		Address offset: 0x10 */
	uint32_t PR;     /* Pending register,          					Address offset: 0x14 */

}EXTI_RegDef_t;



#define EXTI				((EXTI_RegDef_t*)EXTI_BASE)

#endif /* EXTI_H_ */
