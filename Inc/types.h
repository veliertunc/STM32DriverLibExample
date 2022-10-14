/*
 * types.h
 * Contains general purpose type declarations
 *  Created on: Oct 14, 2022
 *      Author: Veli Ertunç
 */

#ifndef TYPES_H_
#define TYPES_H_

#include <stddef.h>
#include <stdint.h>

#define _vu32 volatile uint32_t

#define TRUE				1
#define FALSE				0
#define ENABLE				TRUE
#define DISABLE				FALSE
#define SET					TRUE
#define RESET				FALSE

typedef volatile struct
{
	uint32_t MODER;                         /* Port mode register,          	          	Address offset: 0x00      */
	uint32_t OTYPER;                        /* Output type register,     					Address offset: 0x04      */
	uint32_t OSPEEDR;					    /* Output speed register,     					Address offset: 0x08      */
	uint32_t PUPDR;						    /* Output pull up/down register,				Address offset: 0x0C      */
	uint32_t IDR;						    /* Input data register,     					Address offset: 0x10      */
	uint32_t ODR;						    /* Output data register,     					Address offset: 0x04      */
	uint32_t BSRR;							/* Bit set/reset register,     					Address offset: 0x04      */
	uint32_t LCKR;							/* Configuration lock register,     			Address offset: 0x04      */
	uint32_t AFR[2];					 	/* AFR[0] : GPIO alternate function low register, AFR[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;




#endif /* TYPES_H_ */
