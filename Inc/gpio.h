/*
 * gpio.h
 *
 *  Created on: Oct 14, 2022
 *      Author: Veli Ertunç
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "baseaddr.h"
#include "exti.h"
#include "syscfg.h"
#include "nvic.h"
#include "rcc.h"

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
/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN					  0		/* Input mode						 		*/
#define GPIO_MODE_OUT 					  1		/* Output mode						 		*/
#define GPIO_MODE_AF	 				  2		/* Alternate function mode			 		*/
#define GPIO_MODE_ANALOG 				  3		/* Analog mode						 		*/
#define GPIO_MODE_IT_FT     			  4		/* Interrupt with Falling trigger			*/
#define GPIO_MODE_IT_RT     			  5		/* Interrupt with Rising trigger			*/
#define GPIO_MODE_IT_RFT    			  6		/* Interrupt with Falling-Rising trigger	*/

/*
 * GPIO pin possible output types
 */
#define GPIO_OTYPE_PUSH_PULL			   0
#define GPIO_OTYPE_OPEN_DRAIN			   1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW						0
#define GPIO_SPEED_MEDIUM					1
#define GPIO_SPEED_FAST						2
#define GPOI_SPEED_HIGH						3

/*
 * Pull up/down configuration
 */
#define GPIO_PULL_NONE 						0
#define GPIO_PULL_UP						1
#define GPIO_PULL_DOWN						2

/**Sets peripheral clock of the port
 *  @param pGPIOx: The pointer to the port
 *  @param enabled: Set ENABLE to enable the clock, DISABLE otherwise
 */
void GPIO_SetPeripheralClock(GPIO_RegDef_t* pGPIOx, uint8_t enabled);

/**Initializes the pin with configuration given in handle
 *	@param pHandle: The handle for pin
 */
void GPIO_Init(GPIO_Handle_t* pHandle);

/**Resets GPIO registers to their initial values
 *  @param pGPIOx: The pointer to the port
 */
void GPIO_Reset(GPIO_RegDef_t* pGPIOx);

/**Reads the pin
 *	@param pGPIOx: The pointer to the port
 *	@param pin: The pin number to read
 *	@returns HIGH or LOW
 */
uint32_t GPIO_Read(GPIO_RegDef_t* pGPIOx, uint8_t pin);

/**Reads the port
 *  @param pGPIOx: The pointer to the port
 *  @returns The status of all pins
 */
uint32_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx);

/**Writes value to the pin
 * @param pGPIOx: The pointer to the port
 * @param pin: The pin to write value
 * @param value: The value to write, either HIGH or LOW
 */
void GPIO_Write(GPIO_RegDef_t* pGPIOx, uint8_t pin, uint8_t value);

/**Writes value to the port
 * @param pGPIOx: The pointer to the port
 * @param value: The value to write.
 */
void GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint32_t value);

/**Toggles the pin
 * @param pGPIOx: The pointer to the port
 * @param pin: The pin to toggle.
 */
void GPIO_Toggle(GPIO_RegDef_t* pGPIOx, uint8_t pin);

/*
 * ISR handling
 */
void GPIO_HandleIRQ(uint8_t pin);

#endif /* GPIO_H_ */
