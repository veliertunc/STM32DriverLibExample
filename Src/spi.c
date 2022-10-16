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
