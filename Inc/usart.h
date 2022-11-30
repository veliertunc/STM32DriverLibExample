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
	uint32_t TxLen;
	uint32_t RxLen;
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

/*Options for Length in config structure*/
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



#endif /* USART_H_ */
