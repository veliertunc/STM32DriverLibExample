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

#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4



/*
 * @SPI_Mode
 */
#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0


/*
 * @SPI_Config
 */
#define SPI_BUS_CONFIG_FD                1
#define SPI_BUS_CONFIG_HD                2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128             	6
#define SPI_SCLK_SPEED_DIV256             	7

/*
 * @SPI_DataFormat
 */
#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS  1

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH		 1
#define SPI_CPOL_LOW		 0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN     1
#define SPI_SSM_DI     0

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)

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

/**Enables or disables the specified peripheral
 * @param pSPIx: The pointer to SPI peripheral
 * @param isEnable: If ENABLE enables te peripheral, disables otherwise
 */
void SPI_SetPeripheral(SPI_RegDef_t* pSPIx,uint8_t isEnable);

/**Initializes the given SPI peripheral
 * @param pHandle: The pointer to the handle structure
 */
void SPI_Init(SPI_Handle_t *pHandle);

/**Sends data in buffer
 * @param pSPIx: The pointer to the SPI peripheral
 * @param pTxBuffer: The data to send
 * @param len: The length of the data sent
 * @return none
 */
void SPI_Send(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len);

/**Sends data in buffer
 * @param pSPIx: The pointer to the SPI peripheral
 * @param pRxBuffer: The data to receive
 * @param len: The length of the data received
 * @return none
 */
void SPI_Receive(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t len);


/**Sends data in buffer
 * @param pSPIx: The pointer to the handle
 * @param pTxBuffer: The data to send
 * @param len: The length of the data sent
 * @return The state of transmission
 */
uint8_t SPI_SendIT(SPI_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t len);

/**Receives data from buffer
 * @param pHandle: The pointer to the handle
 * @param pRxBuffer: The data to receive
 * @param len: The length of the data received
 * @return The state of reception
 */
uint8_t SPI_ReceiveIT(SPI_Handle_t* pHandle, uint8_t* pRxBuffer, uint32_t len);

/**Get the value of the specified flag
 * @param pSPIx: The pointer to the SPIx peripheral
 * @param flag: The flag to check
 * @return SET if flag bit is 1, RESET otherwise.
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t flag);

/**Closes the SPI transmission
 * @param pHandle: The pointer to the handle
 * @return none
 */
void SPI_CloseTransmisson(SPI_Handle_t *pHandle);

/**Closes the SPI reception
 * @param pHandle: The pointer to the handle
 * @return none
 */
void SPI_CloseReception(SPI_Handle_t *pHandle);

/**Clears OVR
 * @param pSPIx: The pointer to SPIx peripheral
 * @return none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)

/**Sets SSI bit of the peripheral
 * @param pSPIx: The pointer to SPIx peripheral
 * @param isEnable: If ENABLE SS pin internally connected to ground.
 * @return none
 */
void SPI_SetSSI(SPI_RegDef_t *pSPIx, uint8_t isEnable);

/**Sets SSOE bit of the peripheral
 * @param pSPIx: The pointer to SPIx peripheral
 * @param isEnable: If ENABLE enables SS output
 * @return none
 */
void SPI_SetSSOE(SPI_RegDef_t *pSPIx, uint8_t isEnable);

/*
 * ISR handling
 */
void SPI_HandleIRQ(SPI_Handle_t* pHandle);

static void SPI_HandleTxInterrupt(SPI_Handle_t* pHandle);
static void SPI_HandleRxInterrupt(SPI_Handle_t* pHandle);
static void SPI_HandleOVRErrInterrupt(SPI_Handle_t* pHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t event);

#endif /* SPI_H_ */
