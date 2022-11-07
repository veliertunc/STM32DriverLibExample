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

typedef struct
{
	uint32_t ClockSpeed;
	uint8_t  DeviceAddress;
	uint8_t  AckControl;
	uint8_t  FMDutyCycle;

}I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	config;
	uint8_t 		*pTxBuffer; /* Tx buffer address */
	uint8_t 		*pRxBuffer;	/* Rx buffer address */
	uint32_t 		TxLen;		/* Tx len */
	uint32_t 		RxLen;		/* Rx len */
	uint8_t 		TxRxState;	/* Communication state */
	uint8_t 		DevAddr;	/* Slave/device address */
    uint32_t        RxSize;		/* Rx size */
    uint8_t         Sr;			/* Repeated start value */
}I2C_Handle_t;

/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000


/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE        1
#define I2C_ACK_DISABLE       0


/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2        0
#define I2C_FM_DUTY_16_9     1


/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET


/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

void I2C_SetPeripheralClock(I2CRegDef_t *pI2Cx,uint8_t isEnable);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t flag);
void I2C_SetAcking(I2C_RegDef_t *pI2Cx, uint8_t isEnable);
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_AddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
void I2C_AddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
void I2C_ClearADDRFlag(I2C_Handle_t *pHandle);

void I2C_Init(I2C_Handle_t *pHandle);


#endif /* I2C_H_ */
