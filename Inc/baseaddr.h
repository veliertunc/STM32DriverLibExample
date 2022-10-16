/*
 * baseaddr.h
 * Contains base addresses/register addresses for peripherals.
 *  Created on: Oct 14, 2022
 *      Author: Veli Ertunç
 */

#ifndef BASEADDR_H_
#define BASEADDR_H_
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

/*
 * Base addresses of Flash,SRAMs and ROM
 */
#define FLASH_BASE						0x08000000U   		/* Start address of Flash memory */
#define SRAM1_BASE						0x20000000U  		/* Start address of SRAM1 memory */
#define SRAM2_BASE						0x2001C000U 		/* Start address of SRAM2 memory */
#define ROM_BASE						0x1FFF0000U 		/* Start address of ROM memory */

/*
 * Base addresses of AHB, APB buses and Peripheral
 */
#define PERIPH_BASE 					0x40000000U
#define APB1PERIPH_BASE					PERIPH_BASE
#define APB2PERIPH_BASE					0x40010000U
#define AHB1PERIPH_BASE					0x40020000U
#define AHB2PERIPH_BASE					0x50000000U

/*Base addresses of GPIO ports*/
#define GPIOA_BASE                   	(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE                  	(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE 						(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE 						(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE 						(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE 						(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE 						(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE 						(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE 						(AHB1PERIPH_BASE + 0x2000)
#define RCC_BASE                    	(AHB1PERIPH_BASE + 0x3800)

/*Base addresses of SPI peripherals*/
#define SPI1_BASE						(APB2PERIPH_BASE + 0x3000)
#define SPI2_BASE						(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE						(APB1PERIPH_BASE + 0x3C00)

#define EXTI_BASE						(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE	        			(APB2PERIPH_BASE + 0x3800)
#define NVIC_PR_BASE				 	((_vu32*)0xE000E400)


#endif /* BASEADDR_H_ */
