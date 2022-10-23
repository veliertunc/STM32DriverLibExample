/*
 * spi.c
 *
 *  Created on: 15 Oct 2022
 *      Author: Veli Ertunç
 */
#include "spi.h"


void SPI_SetPeripheralClock(SPI_RegDef_t* pSPIx, uint8_t enabled)
{
	if (enabled) {
		switch ((uint32_t)pSPIx) {
			case (uint32_t)SPI1:
				RCC->APB2ENR |= (1 << 12);
				break;
			case (uint32_t)SPI2:
				RCC->APB1ENR |= (1 << 14);
				break;
			case (uint32_t)SPI3:
				RCC->APB1ENR |= (1 << 15);
				break;
			default:
				break;
		}
	} else {
		switch ((uint32_t)pSPIx) {
			case (uint32_t)SPI1:
				RCC->APB2RSTR |= (1 << 12);
				RCC->APB2RSTR &= ~(1 << 12);
				break;
			case (uint32_t)SPI2:
				RCC->APB1RSTR |= (1 << 14);
				RCC->APB1RSTR &= ~(1 << 14);
				break;
			case (uint32_t)SPI3:
				RCC->APB1RSTR |= (1 << 15);
				RCC->APB1RSTR &= ~(1 << 15);
				break;
			default:
			break;
		}
	}
}

void SPI_SetPeripheral(SPI_RegDef_t* pSPIx,uint8_t isEnable)
{
	if (isEnable) {
		pSPIx->CR1 |= (1U << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1U << SPI_CR1_SPE);
	}
}

void SPI_Init(SPI_Handle_t *pHandle)
{
	//1. Enable the SPI peripheral clock
	SPI_SetPeripheralClock(pHandle->pSPIx, ENABLE);

	uint8_t mode = pHandle->config.Mode;
	uint8_t busConfig = pHandle->config.Config;
	uint8_t clkSpeed = pHandle->config.ClockSpeed;
	uint8_t dff = pHandle->config.DataFormat;
	uint8_t cpol = pHandle->config.Polarity;
	uint8_t cpha = pHandle->config.Phase;
	uint8_t ssm = pHandle->config.SSM;

	//2. Select device mode
	pHandle->pSPIx->CR1 |= (mode << SPI_CR1_MSTR );

	//3. Select bus configuration
	if(busConfig == SPI_BUS_CONFIG_FD)
	{
		/*Bit 15 BIDIMODE: Bidirectional data mode enable
			0: 2-line unidirectional data mode selected
			1: 1-line bidirectional data mode selected
		*/
		pHandle->pSPIx->CR1 &= ~( 1 << SPI_CR1_BIDIMODE);//Clear bit 15
	}else if (busConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		pHandle->pSPIx->CR1 |= ( 1 << SPI_CR1_BIDIMODE);//Set bit 15
	}else if (busConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		pHandle->pSPIx->CR1 &= ~( 1 << SPI_CR1_BIDIMODE);//Clear bit 15
		//RXONLY bit must be set
		pHandle->pSPIx->CR1 |= ( 1 << SPI_CR1_RXONLY);//And set bit 10
	}

	//4. Set SCLK speed
	pHandle->pSPIx->CR1 |= (clkSpeed << SPI_CR1_BR);
	//5. Configure data format (8 bit or 16 bit)
	pHandle->pSPIx->CR1 |= (dff << SPI_CR1_DFF);
	//6. Configure polarity
	pHandle->pSPIx->CR1 |= (cpol << SPI_CR1_CPOL);
	//7. Configure phase
	pHandle->pSPIx->CR1 |= (cpha << SPI_CR1_CPHA);
	//8. Configure SS Management
	pHandle->pSPIx->CR1 |= (ssm << SPI_CR1_SSM);
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t flag)
{
	if(pSPIx->SR & flag)
	{
		return SET;
	}
	return RESET;
}

void SPI_Send(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint_32_t len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == RESET );

		//2. check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR =   *((uint16_t*)pTxBuffer);
			len-=2;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR =   *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

void SPI_Receive(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint_32_t len)
{
	while(len > 0)
	{
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

		//2. check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load the data from DR to Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			len-=2;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->DR ;
			len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendIT(SPI_Handle_t* pHandle, uint8_t* pTxBuffer, uint32_t len)
{
	if (pHandle->TxState != SPI_BUSY_IN_TX) {
		//1. Set SPI Tx buffer to pTxBuffer, and TxLen to len
		pHandle->pTxBuffer = pTxBuffer;
		pHandle->TxLen = len;
		//2. Change TxState
		pHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enable TX Interrupt by setting TXEIE bit in CR2 register
		pHandle->pSPIx->CR2 |= (1U << SPI_CR2_TXEIE);
	}
	return pHandle->TxState;
}

uint8_t SPI_ReceiveIT(SPI_Handle_t* pHandle, uint8_t* pRxBuffer, uint32_t len)
{
	if (pHandle->RxState != SPI_BUSY_IN_RX) {
		//1. Set SPI Rx buffer to pRxBuffer, and RxLen to len
		pHandle->pRxBuffer = pRxBuffer;
		pHandle->RxLen = len;
		//2. Change TxState
		pHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable TX Interrupt by setting RXNEIE bit in CR2 register
		pHandle->pSPIx->CR2 |= (1U << SPI_CR2_RXNEIE);
	}
}

void SPI_SetSSI(SPI_RegDef_t *pSPIx, uint8_t isEnable)
{
	if (isEnable) {
		pSPIx->CR1 |= (1U << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1U << SPI_CR1_SSI);
	}
}

void SPI_SetSSOE(SPI_RegDef_t *pSPIx, uint8_t isEnable)
{
	if (isEnable) {
		pSPIx->CR2 |= (1U << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1U << SPI_CR2_SSOE);
	}
}

void SPI_CloseTransmisson(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	/* From RM0090 - Reference Manual
	* Clearing the OVR bit is done by a read operation on the SPI_DR register
	* followed by a read access to the SPI_SR register
	*/
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


void SPI_HandleIRQ(SPI_Handle_t* pHandle)
{
	uint32_t sr = pHandle->pSPIx->SR;
	uint32_t cr2 = pHandle->pSPIx->CR2;
	uint8_t status = sr & (1<<SPI_SR_TXE);
	uint8_t flag = cr2 & (1<< SPI_CR2_TXEIE);

	if (status && flag) {
		//Tx interrupt occured
		SPI_HandleTxInterrupt(pHandle);
	}

	status = sr & (1 << SPI_SR_RXNE);
	flag = cr2 & (1 << SPI_CR2_RXNEIE);

	if (status && flag) {
		//Rx interrupt occured
		SPI_HandleRxInterrupt(pHandle);
	}

	status = sr & (1 << SPI_SR_OVR);
	flag = cr2 & (1 << SPI_CR2_ERRIE);

	if (status && flag) {
		//OVR Error occured
		SPI_HandleOVRErrInterrupt(pHandle);
	}

}

static void SPI_HandleTxInterrupt(SPI_Handle_t* pHandle)
{
	uint32_t cr1 = pHandle->pSPIx->CR1;
	uint32_t dataFormat = cr1 & (1 << SPI_CR1_DFF);
	if (dataFormat) {
		//16 bit data format
		pHandle->pSPIx->DR = *((uint16_t*)pHandle->pTxBuffer);//Load 2 bytes from TxBuffer into DataRegister
		pHandle->TxLen-=2;//We sent 2 bytes above so decrement TxLen by 2.
		(uint16_t*)(pHandle->pTxBuffer)++;
	} else {
		//8 bit data format
		pHandle->pSPIx->DR = *pHandle->pTxBuffer;//Load TxBuffer into DataRegister
		pHandle->TxLen--;
		pHandle->pTxBuffer++;
	}

	if (!pHandle->TxLen) {
		//No data remained
		SPI_CloseTransmisson(pHandle);
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);
	}
}

static void SPI_HandleRxInterrupt(SPI_Handle_t* pHandle)
{
	uint32_t cr1 = pHandle->pSPIx->CR1;
	uint32_t dataFormat = cr1 & (1 << SPI_CR1_DFF);
	if (dataFormat) {
		//16 bit data format
		*((uint16_t*)pHandle->pRxBuffer) = pHandle->pSPIx->DR; //Load 2 bytes from DR into RxBuffer
		pHandle->RxLen-=2;//We sent 2 bytes above so decrement TxLen by 2.
		(uint16_t*)(pHandle->pRxBuffer)++;
	} else {
		//8 bit data format
		*pHandle->pRxBuffer = pHandle->pSPIx->DR;//Load DR into RxBuffer
		pHandle->RxLen--;
		pHandle->pRxBuffer++;
	}

	if (!pHandle->RxLen) {
		//No data remained
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void SPI_HandleOVRErrInterrupt(SPI_Handle_t* pHandle)
{
	//1. clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pHandle->pSPIx);
	}
	//2. inform the application
	SPI_ApplicationEventCallback(pHandle,SPI_EVENT_OVR_ERR);
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle,uint8_t event)
{

}
