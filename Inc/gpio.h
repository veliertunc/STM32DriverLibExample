/*
 * gpio.h
 *
 *  Created on: Oct 14, 2022
 *      Author: Veli Ertunç
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "types.h"
#include "baseaddr.h"

#define GPIOA  							((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB  							((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC  							((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD  							((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE  							((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF  							((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG  							((GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH  							((GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI  							((GPIO_RegDef_t*)GPIOI_BASE)


#define GPIO_TO_CODE(x)      		   	((x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7:\
								        (x == GPIOI)?8:0)


void Gpio_SetPeripheralClock(GPIO_RegDef_t* GPIOx, uint8_t enabled);

void Gpio_Reset(GPIO_RegDef_t* GPIOx);



#endif /* GPIO_H_ */
