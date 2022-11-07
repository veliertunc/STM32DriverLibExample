/*
 * i2c.c
 *
 *  Created on: 23 Oct 2022
 *      Author: Veli Ertunç
 */

#include "i2c.h"

void I2C_SetPeripheralClock(I2CRegDef_t *pI2Cx,uint8_t isEnable)
{
	if (isEnable) {
		if(pI2Cx == I2C1)
		{
			RCC->APB1ENR |= (1U << 21);
		}
		if(pI2Cx == I2C2)
		{
			RCC->APB1ENR |= (1U << 22);
		}

		if(pI2Cx == I2C3)
		{
			RCC->APB1ENR |= (1U << 23);
		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			RCC->APB1ENR &= ~(1U << 21);
		}
		if(pI2Cx == I2C2)
		{
			RCC->APB1ENR &= ~(1U << 22);
		}

		if(pI2Cx == I2C3)
		{
			RCC->APB1ENR &= ~(1U << 23);
		}
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t flag)
{
	return (pI2Cx->SR1 & flag)? FLAG_SET:FLAG_RESET;
}

void I2C_SetAcking(I2C_RegDef_t *pI2Cx, uint8_t isEnable)
{
	if(isEnable)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1U << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1U << I2C_CR1_ACK);
	}
}

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1U << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1U << I2C_CR1_STOP);
}

void I2C_Init(I2C_Handle_t *pHandle)
{
	uint32_t clkSpeed = pHandle->config.ClockSpeed;
	uint8_t  devAddr = pHandle->config.DeviceAddress;
	uint8_t  ackControl = pHandle->config.AckControl;
	uint8_t  dutyCycle = pHandle->config.FMDutyCycle;

	//1. Enable peripheral clock
	I2C_SetPeripheralClock(pHandle->pI2Cx, ENABLE);
	//2. Set ACK control bit
	pHandle->pI2Cx->CR1 = (ackControl << 10);
	//3.Configure FREQ field of CR2
	uint32_t clk1=RCC_GetPCLK1Value()/1000000U;
	pHandle->pI2Cx->CR2 = (clk1 & 0x3F);
	//4. Set device address
	pHandle->pI2Cx->OAR1 = (1U << 14) | (devAddr << 1);
	//5. Configure CCR depending on mode
	if (clkSpeed <= I2C_SCL_SPEED_SM) {
		//Standard mode
		uint32_t val = RCC_GetPCLK1Value()/(2*clkSpeed);
		pI2CHandle->pI2Cx->CCR = val & 0xFFF;
		//Set TRISE register
		pHandle->pI2Cx->TRISE = (clk1+1) & 0x3F;
	} else {
		//Fast mode
		uint32_t val = (1U << 15) | (dutyCycle << 14);
		if (dutyCycle == I2C_FM_DUTY_2) {
			val |= (RCC_GetPCLK1Value() / (3*clkSpeed) );
		} else {
			val |= (RCC_GetPCLK1Value() / (25*clkSpeed) );
		}
		pHandle->pI2Cx->CCR = val&0xFFF;
		//Set TRISE register
		pHandle->pI2Cx->TRISE = (clk1*3/10+1) & 0x3F;
	}
}

void I2C_AddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr &= ~(1); //slaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = slaveAddr;
}

void I2C_AddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr |= 1; //slaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = slaveAddr;
}

void I2C_ClearADDRFlag(I2C_Handle_t *pHandle)
{
	uint32_t dummy_read;
	//check for device mode
	if(pHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_SetAcking(pHandle->pI2Cx,DISABLE);
			}
		}
	}
	//clear the ADDR flag ( read SR1 , read SR2)
	dummy_read = pHandle->pI2Cx->SR1;
	dummy_read = pHandle->pI2Cx->SR2;
	(void)dummy_read;
}
