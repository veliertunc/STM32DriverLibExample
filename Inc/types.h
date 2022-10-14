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
#define ENABLE				1
#define DISABLE				0
#define SET					1
#define RESET				0
#define HIGH				1
#define LOW					0

typedef volatile struct
{
	uint32_t MEMRMP;       /* Remap register,                   			Address offset: 0x00      */
	uint32_t PMC;          /* Peripheral mode configuration register,     	Address offset: 0x04      */
	uint32_t EXTICR[4];    /* External interrupt configuration registers,	Address offset: 0x08-0x14 */
	uint32_t RESERVED1[2]; /* Reserved, 									Address offset: 0x18-0x1C */
	uint32_t CMPCR;        /* Compensation cell control register,         	Address offset: 0x20      */
	uint32_t RESERVED2[2]; /* Reserved, 									Address offset: 0x24-0x28 */
}SYSCFG_RegDef_t;

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

/*
 * The structure used to configure a pin
 */
typedef struct
{
	uint8_t PinNo;
	uint8_t Mode;			/* Possible values from @GPIO_PIN_MODES >*/
	uint8_t Speed;			/* Possible values from @GPIO_PIN_SPEED >*/
	uint8_t Pull;
	uint8_t OutputType;
	uint8_t AltFunMode;
}GPIO_PinConfig_t;

/*
 * The handle structure for a pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;       		/* Holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t config;   			/* Holds GPIO pin configuration settings >*/
}GPIO_Handle_t;



#endif /* TYPES_H_ */
