/*
 * i2c.h
 *
 *  Created on: 23 Oct 2022
 *      Author: Veli Ertunç
 */

#ifndef I2C_H_
#define I2C_H_
#include "baseaddr.h"
#include "rcc.h"
#include "nvic.h"
#include "syscfg.h"
#include "exti.h

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/*I2C register definition structure*/
typedef volatile struct
{
  uint32_t CR1;        /*  Control register 1,     									Address offset: 0x00 */
  uint32_t CR2;        /*  Control register 2,     									Address offset: 0x04 */
  uint32_t OAR1;       /*  Own address register 1,     								Address offset: 0x08 */
  uint32_t OAR2;       /*  Own address register 2,     								Address offset: 0x0C */
  uint32_t DR;         /*  Data register,     										Address offset: 0x10 */
  uint32_t SR1;        /*  Status register 1,     									Address offset: 0x14 */
  uint32_t SR2;        /*  Status register 2,     									Address offset: 0x18 */
  uint32_t CCR;        /*  Clock control register,     								Address offset: 0x1C */
  uint32_t TRISE;      /*  TRISE register,     										Address offset: 0x20 */
  uint32_t FLTR;       /*  FLTR register,     										Address offset: 0x24 */
}I2C_RegDef_t;

#define I2C1  				((I2C_RegDef_t*)I2C1_BASE)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASE)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASE)

void SetPeripheralClock(I2CRegDef_t *pI2Cx,uint8_t isEnable);

#endif /* I2C_H_ */
