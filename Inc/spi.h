/*
 * spi.h
 *
 *  Created on: 15 Oct 2022
 *      Author: Veli Ertunç
 */

#ifndef SPI_H_
#define SPI_H_
#include "baseaddr.h"
#include "rcc.h"
#include "nvic.h"
#include "syscfg.h"
#include "exti.h"

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/**
 * Register definition for SPI peripheral
 */
typedef volatile struct {
	uint32_t CR1;			/* Control register 1,     							Address offset: 0x00 */
	uint32_t CR2; 			/* Control register 2,     							Address offset: 0x04 */
	uint32_t SR; 			/* Status register,     							Address offset: 0x08 */
	uint32_t DR; 			/* Data register,     								Address offset: 0x0C */
	uint32_t CRCPR; 		/* CRC polynomial register,     					Address offset: 0x10 */
	uint32_t RXCRCR;		/* RX CRC register,     							Address offset: 0x14 */
	uint32_t TXCRCR;		/* TX CRC register,     							Address offset: 0x18 */
	uint32_t I2SCFGR; 		/* I2S configuration register,     					Address offset: 0x1C */
	uint32_t I2SPR; 		/* I2S prescaler register,     						Address offset: 0x20 */
} SPI_RegDef_t;

#define SPI1  				((SPI_RegDef_t*)SPI1_BASE)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASE)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASE)

/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct {
	uint8_t Mode; 			/* Master or slave 		*/
	uint8_t Config; 		/* Bus configuration 	*/
	uint8_t ClockSpeed;
	uint8_t DataFormat; 	/* 8 bit or 16 bit		*/
	uint8_t Polarity;		/* CPOL bit 			*/
	uint8_t Phase; 			/* CPHA bit				*/
	uint8_t SSM; 			/* SSM bit 				*/
} SPI_Config_t;

/*
 *Handle structure for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t *pSPIx; 		/* Holds the base address of SPIx(x:0,1,2) peripheral	*/
	SPI_Config_t config;
	uint8_t *pTxBuffer; 		/* Holds Tx buffer address								*/
	uint8_t *pRxBuffer; 		/* Holds Rx buffer address								*/
	uint32_t TxLen; 			/* Tx len												*/
	uint32_t RxLen; 			/* Rx len				 	 							*/
	uint8_t TxState; 			/* Tx State 											*/
	uint8_t RxState; 			/* Rx State					 							*/
} SPI_Handle_t;

/**Sets peripheral clock of the SPIx
 *  @param pSPIx: The pointer to the SPI peripheral
 *  @param enabled: Set ENABLE to enable the clock, DISABLE otherwise
 */
void SPI_SetPeripheralClock(SPI_RegDef_t *pSPIx, uint8_t enabled);

/**Initializes the given SPI peripheral
 * @param pHandle: The pointer to the handle structure
 */
void SPI_Init(SPI_Handle_t *pHandle);

#endif /* SPI_H_ */
