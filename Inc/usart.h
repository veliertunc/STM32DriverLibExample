/*
 * usart.h
 *
 *  Created on: 30 Nov 2022
 *      Author: Veli Ertunç
 */

#ifndef USART_H_
#define USART_H_
#include "baseaddr.h"
#include "rcc.h"
#include "nvic.h"
#include "syscfg.h"

typedef volatile struct
{
	uint32_t SR;         /* Status register,     									Address offset: 0x00 */
	uint32_t DR;         /* Data register,     										Address offset: 0x04 */
	uint32_t BRR;        /* Baud rate register,     								Address offset: 0x08 */
	uint32_t CR1;        /* Control register 1,     								Address offset: 0x0C */
	uint32_t CR2;        /* Control register 2,     								Address offset: 0x10 */
	uint32_t CR3;        /* Control register 3,     								Address offset: 0x14 */
	uint32_t GTPR;       /* Guard time and prescaler register,     					Address offset: 0x18 */
} USART_RegDef_t


#define USART1  			((USART_RegDef_t*)USART1_BASE)
#define USART2  			((USART_RegDef_t*)USART2_BASE)
#define USART3  			((USART_RegDef_t*)USART3_BASE)
#define UART4  				((USART_RegDef_t*)UART4_BASE)
#define UART5  				((USART_RegDef_t*)UART5_BASE)
#define USART6  			((USART_RegDef_t*)USART6_BASE)

typedef struct
{
	uint8_t Mode;
	uint32_t Baud;
	uint8_t StopBits;
	uint8_t Length;
	uint8_t Parity;
	uint8_t FlowControl;
}USART_Config_t;

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t Txlen;
	uint32_t Rxlen;
	uint8_t TxState;
	uint8_t RxState;
}USART_Handle_t;

/*Options for Mode in config structure*/
#define USART_MODE_TX 				0
#define USART_MODE_RX	 			1
#define USART_MODE_TXRX  			2

/*Options for Parity in config structure*/
#define USART_PARITY_NONE			0
#define USART_PARITY_EVEN			1
#define USART_PARITY_ODD			2

/*Options for length in config structure*/
#define USART_LENGTH_8BITS			0
#define USART_LENGTH_9BITS			1

/*Options for StopBits in config structure*/
#define USART_STOPBITS_1			0
#define USART_STOPBITS_0_5			1
#define USART_STOPBITS_2			2
#define USART_STOPBITS_1_5			3

/*Options for FlowControl in config structure*/
#define USART_FLOW_CTRL_NONE		0
#define USART_FLOW_CTRL_CTS			1
#define USART_FLOW_CTRL_RTS			2
#define USART_FLOW_CTRL_CTS_RTS		3

/* USART flags  */
#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

#define 	USART_EVENT_TX_CMPLT   		0
#define		USART_EVENT_RX_CMPLT   		1
#define		USART_EVENT_IDLE      		2
#define		USART_EVENT_CTS       		3
#define		USART_EVENT_PE       		4
#define		USART_ERR_FE    		 	5
#define		USART_ERR_NE   				6
#define		USART_ERR_ORE    			7

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/
/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */
#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

void USART_SetPeripheralClock(USART_RegDef_t *pUSARTx, uint8 isEnable);

void USART_Init(USART_Handle_t *pHandle);

void USART_Send(USART_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len);
void  USART_Receive(USART_Handle_t *pHandle,uint8_t *pRxBuffer, uint32_t len);
uint8_t USART_SendIT(USART_Handle_t *pHandle,uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveIT(USART_Handle_t *pHandle,uint8_t *pRxBuffer, uint32_t len);

void USART_HandleIRQ(USART_Handle_t *pHandle);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t flag);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag);
void USART_EnablePeripheral(USART_RegDef_t *pUSARTx, uint8_t isEnable);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baud);

void USART_AppEventCallback(USART_Handle_t *pHandle,uint8_t evt);

#endif /* USART_H_ */
