/*
 * usart.c
 *
 *  Created on: 30 Nov 2022
 *      Author: Veli Ertunç
 */
#include "usart.h"

void USART_SetPeripheralClock(USART_RegDef_t *pUSARTx, uint8 isEnable)
{
	if (isEnable) {
		if(pUSARTx == USART1)
		{
			RCC->APB2ENR |= (1 << 4);
		}else if (pUSARTx == USART2)
		{
			RCC->APB1ENR |= (1 << 17);
		}else if (pUSARTx == USART3)
		{
			RCC->APB1ENR |= (1 << 18);
		}
		else if (pUSARTx == UART4)
		{
			RCC->APB1ENR |= (1 << 19);
		}else if(pUSARTx == UART5)
		{
			RCC->APB1ENR |= (1 << 20);
		}else if(pUSARTx == USART6)
		{
			RCC->APB2ENR |= (1 << 5);
		}
	}else
	{
		if(pUSARTx == USART1)
		{
			RCC->APB2ENR &= ~(1 << 4);
		}else if (pUSARTx == USART2)
		{
			RCC->APB1ENR &= ~(1 << 17);
		}else if (pUSARTx == USART3)
		{
			RCC->APB1ENR &= ~(1 << 18);
		}
		else if (pUSARTx == UART4)
		{
			RCC->APB1ENR &= ~(1 << 19);
		}else if(pUSARTx == UART5)
		{
			RCC->APB1ENR &= ~(1 << 20);
		}else if(pUSARTx == USART6)
		{
			RCC->APB2ENR &= ~(1 << 5);
		}
	}
}

void USART_Init(USART_Handle_t *pHandle)
{
	uint32_t reg=0;
	uint8_t mode = pHandle->config.Mode;
	uint32_t baud = pHandle->config.Baud;
	uint8_t stop = pHandle->config.StopBits;
	uint8_t len = pHandle->config.Length;
	uint8_t parity = pHandle->config.Parity;
	uint8_t flowCtrl = pHandle->config.FlowControl;

	//1. Set peripheral clock
	USART_SetPeripheralClock(pHandle->pUSARTx, ENABLE);
	//2. Set USART mode
	if(mode == USART_MODE_RX)
	{
		reg |= (1 << USART_CR1_RE);
	}else if(mode == USART_MODE_TX)
	{
		reg |= (1 << USART_CR1_TE);
	}else if(mode == USART_MODE_TXRX)
	{
		reg |= (1 << USART_CR1_RE) | (1 << USART_CR1_TE);
	}
	//3. Set data length (8bits/9bits)
	reg |= (len << USART_CR1_M);
	//4. Set parity control
	if (parity == USART_PARITY_EVEN) {
		reg |= (1 << USART_CR1_PCE);//Enable parity control
	}else if (parity == USART_PARITY_ODD) {
		reg |= (1 << USART_CR1_PCE);//Enable parity control
		reg |= (1 << USART_CR1_PS);//0-> EVEN, 1-> ODD
	}

	pHandle->pUSARTx->CR1 = reg;
	reg = 0;

	reg |= stop << USART_CR2_STOP;
	pHandle->pUSARTx->CR2 = reg;

	reg = 0;
	if (flowCtrl == USART_FLOW_CTRL_CTS) {
		reg |= (1 << USART_CR3_CTSE);
	}else if (flowCtrl == USART_FLOW_CTRL_RTS) {
		reg |= (1 << USART_CR3_RTSE);
	}else if (flowCtrl == USART_FLOW_CTRL_CTS_RTS) {
		reg |= (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE);
	}

	pHandle->pUSARTx->CR3 = reg;

	USART_SetBaudRate(pHandle->pUSARTx, baud);
}

void USART_Send(USART_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint16_t *pData;
	uint32_t i;
	for (i = 0; i < len; i++) {
		while(!USART_GetFlagStatus(pHandle->pUSARTx, USART_FLAG_TXE));

		if (pHandle->config.Length == USART_LENGTH_9BITS) {
			pData = (uint16_t*) pTxBuffer;
			pHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);
			if (pHandle->config.Parity == USART_PARITY_NONE) {
				pTxBuffer++;
				pTxBuffer++;
			} else {
				pTxBuffer++;
			}
		} else {
			//8Bits
			pHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}
	//Wait until Transfer complete flag is set
	while( ! USART_GetFlagStatus(pHandle->pUSARTx,USART_FLAG_TC));
}

void  USART_Receive(USART_Handle_t *pHandle,uint8_t *pRxBuffer, uint32_t len)
{
	for(uint32_t i = 0 ; i < len; i++)
	{
		while(! USART_GetFlagStatus(pHandle->pUSARTx,USART_FLAG_RXNE));
		if (pHandle->config.Length == USART_LENGTH_9BITS) {
			if (pHandle->config.Parity == USART_PARITY_NONE) {
				*((uint16_t*) pRxBuffer) = (pHandle->pUSARTx->DR  & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			} else {
				*pRxBuffer = (pHandle->pUSARTx->DR  & (uint8_t)0xFF);
				pRxBuffer++;
			}
		} else {
			if (pHandle->config.Parity == USART_PARITY_NONE) {
				*((uint16_t*) pRxBuffer) = (pHandle->pUSARTx->DR  & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			} else {
				*pRxBuffer = (pHandle->pUSARTx->DR  & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
	}
}

uint8_t USART_SendIT(USART_Handle_t *pHandle,uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pHandle->TxState;
	if (state != USART_BUSY_IN_TX) {
		pHandle->TxLen = Len;
		pHandle->pTxBuffer = pTxBuffer;
		pHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE);
	}
	return state;
}

uint8_t USART_ReceiveIT(USART_Handle_t *pHandle,uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pHandle->RxState;
	if (state != USART_BUSY_IN_RX) {
		pHandle->RxLen = Len;
		pHandle->pRxBuffer = pRxBuffer;
		pHandle->RxState = USART_BUSY_IN_RX;
		(void)pHandle->pUSARTx->DR;
		//Implement the code to enable interrupt for TC
		pHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);
	}
	return state;
}

void USART_HandleIRQ(USART_Handle_t *pHandle)
{
	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

	temp1 = pHandle->pUSARTx->SR & (1 << USART_SR_TC);
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if (temp1 && temp2) {
		if (pHandle->TxState == USART_BUSY_IN_TX) {
			if (pHandle->Txlen == 0) {
				pHandle->pUSARTx->SR &= ~(1<<USART_SR_TC);
				pHandle->TxState = USART_READY;
				pHandle->pTxBuffer = NULL;
				pHandle->Txlen = 0;
				USART_AppEventCallback(pHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}
	temp1 = pHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);
	temp1 = pHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pHandle->pTxBuffer;
					pHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pHandle->pTxBuffer++;
						pHandle->pTxBuffer++;
						pHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pHandle->pTxBuffer++;
						pHandle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pHandle->pUSARTx->DR = (*pHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pHandle->pTxBuffer++;
					pHandle->TxLen-=1;
				}

			}
			if (pHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		if(pHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame
					//Now, check are we using USART_ParityControl control or not
					if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pHandle->pRxBuffer) = (pHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pHandle->pRxBuffer++;
						pHandle->pRxBuffer++;
						pHandle->RxLen-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pHandle->pRxBuffer = (pHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pHandle->pRxBuffer++;
						 pHandle->RxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data
						//read 8 bits from DR
						 *pHandle->pRxBuffer = (uint8_t) (pHandle->pUSARTx->DR  & (uint8_t)0xFF);

					}
					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity
						//read only 7 bits , hence mask the DR with 0X7F
						 *pHandle->pRxBuffer = (uint8_t) (pHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pHandle->pRxBuffer++;
					 pHandle->RxLen-=1;
				}

			}//if of >0

			if(! pHandle->RxLen)
			{
				//disable the rxne
				pHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pHandle->RxBusyState = USART_READY;
				USART_AppEventCallback(pHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		pHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_AppEventCallback(pHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		//this interrupt is because of idle
		USART_AppEventCallback(pHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_AppEventCallback(pHandle,USART_ERR_ORE);
	}

/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_AppEventCallback(pHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NE) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_AppEventCallback(pHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_AppEventCallback(pHandle,USART_ERR_ORE);
		}
	}

}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t flag)
{
    if(pUSARTx->SR & flag)
    {
    	return SET;
    }

   return RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag)
{
	pUSARTx->SR &= ~(flag);
}

void USART_EnablePeripheral(USART_RegDef_t *pUSARTx, uint8_t isEnable)
{
	if(isEnable)
	{
		pUSARTx->CR1 |= (1 << 13);
	}else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold the APB clock
	uint32_t PCLKx;
	uint32_t usartdiv;
	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t reg=0;
	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		PCLKx = RCC_GetPCLK1Value();
	}
	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}else
	{
		//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;
	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	reg |= (usartdiv/100) << 4;
	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);
	}else
	{
		//over sampling by 16
		F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
	}
	//Place the fractional part in appropriate bit position . refer USART_BRR
	reg |= F_part;
	//copy the value of reg in to BRR register
	pUSARTx->BRR = reg;
}

__attribute__((weak)) void USART_AppEventCallback(USART_Handle_t *pHandle,uint8_t evt)
