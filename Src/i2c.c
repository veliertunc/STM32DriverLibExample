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
		pHandle->pI2Cx->CCR = val & 0xFFF;
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

void I2C_MasterSend(I2C_Handle_t *pHandle,uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr,uint8_t sr)
{
	// 1. START Condition
	I2C_GenerateStartCondition(pHandle->pI2Cx);
	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_FLAG_SB));
	// 3. Set r/w bit to write(0)
	I2C_AddressPhaseWrite(pHandle->pI2Cx, slaveAddr);
	//4. Check ADDR bit to confirm address phase is completed
	while(!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_FLAG_ADDR));//Check
	//5. Clear addr flag
	I2C_ClearADDRFlag(pHandle);
	//6. Send data
	while(len > 0)
	{
		while(! I2C_GetFlagStatus(pHandle->pI2Cx,I2C_FLAG_TXE) );
		pHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pHandle->pI2Cx,I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pHandle->pI2Cx,I2C_FLAG_BTF) );

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pHandle->pI2Cx);
}

void I2C_MasterReceive(I2C_Handle_t *pHandle,uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr,uint8_t sr)
{
	// 1. START Condition
	I2C_GenerateStartCondition(pHandle->pI2Cx);
	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_FLAG_SB));
	//3. Set r/w to read(1)
	I2C_AddressPhaseRead(pHandle->pI2Cx, slaveAddr);
	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while(!I2C_GetFlagStatus(pHandle->pI2Cx,I2C_FLAG_ADDR));

	if (len == 1) {
		I2C_SetAcking(pHandle->pI2Cx, I2C_ACK_DISABLE);//Disable acking
		I2C_ClearADDRFlag(pHandle);
		while(!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_FLAG_RXNE));//Is RXNE==1?
		if (sr==I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pHandle->pI2Cx);
		}
		*pRxBuffer = pHandle->pI2Cx->DR;//Load DR register into the buffer
	}

	if (len>1) {
		I2C_ClearADDRFlag(pHandle);
		for (uint32_t i = len; i > 0; i--) {
			while(!I2C_GetFlagStatus(pHandle->pI2Cx, I2C_FLAG_RXNE));//Is RXNE==1?
			if (i==2) {
				I2C_SetAcking(pHandle->pI2Cx, I2C_ACK_DISABLE);
				if (sr==I2C_DISABLE_SR) {
					I2C_GenerateStopCondition(pHandle->pI2Cx);
				}
			}
			*pRxBuffer = pHandle->pI2Cx->DR;//Load DR register into the buffer
			pRxBuffer++;//Move to the next byte
		}
	}

	if(pHandle->config.AckControl == I2C_ACK_ENABLE)
	{
		I2C_SetAcking(pHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}

uint8_t I2C_MasterSendIT(I2C_Handle_t *pHandle,uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr,uint8_t sr)
{
	uint8_t state=pHandle->TxRxState;
	if((state!=I2C_BUSY_IN_TX) && (state!=I2C_BUSY_IN_RX))
	{
		pHandle->pTxBuffer = pTxbuffer;
		pHandle->TxLen = len;
		pHandle->TxRxState = I2C_BUSY_IN_TX;
		pHandle->DevAddr = slaveAddr;
		pHandle->Sr = sr;
		I2C_GenerateStartCondition(pHandle->pI2Cx);
		pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITERREN);
	}
	return state;
}

uint8_t I2C_MasterReceiveIT(I2C_Handle_t *pHandle,uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr,uint8_t sr)
{
	uint8_t state=pHandle->TxRxState;
	if((state!=I2C_BUSY_IN_TX) && (state!=I2C_BUSY_IN_RX))
	{
		pHandle->pRxBuffer = pRxBuffer;
		pHandle->RxLen = len;
		pHandle->TxRxState = I2C_BUSY_IN_RX;
		pHandle->RxSize = len;
		pHandle->DevAddr = slaveAddr;
		pHandle->Sr = sr;
		I2C_GenerateStartCondition(pHandle->pI2Cx);
		pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITERREN);
	}
	return state;
}

//IRQn31
void I2C_HandleIRQEV(I2C_Handle_t *pHandle)
{
	uint32_t temp1, temp2, temp3;
	temp1 = pHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);
	temp3 = pHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);

	//1. Handle For interrupt generated by SB event (master)
	if(temp1 && temp3)
	{
		if (pHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_AddressPhaseWrite(pHandle->pI2Cx, pHandle->DevAddr);
		}

		if (pHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_AddressPhaseRead(pHandle->pI2Cx, pHandle->DevAddr);
		}
	}

	temp3 = pHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	//2. Handle ADDR event
	if(temp1 && temp3)
	{
		I2C_ClearADDRFlag(pHandle);
	}

	temp3  = pHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		if(pHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set .
			if(pHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
			{
				//BTF, TXE = 1
				if(pHandle->TxLen == 0 )
				{
					//1. generate the STOP condition
					if(pHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pHandle->pI2Cx);
					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pHandle);
					//3. notify the application about transmission complete
					I2C_AppEventCallback(pHandle,I2C_EV_TX_CMPLT);
				}
			}
		}else if(pHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//BTF, RXNE = 1
			if(pHandle->RxLen == pHandle->RxSize)
			{
				//1. generate the STOP condition
				if(pHandle->Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pHandle->pI2Cx);
				//2. reset all the member elements of the handle structure.
				I2C_CloseReceiveData(pHandle);
				//3. notify the application about transmission complete
				I2C_AppEventCallback(pHandle,I2C_EV_RX_CMPLT);
			}
		}
	}

	temp3  = pHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle STOPF event(in slave mode)
	if(temp1 && temp3)
	{
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )
		pHandle->pI2Cx->CR1 |= 0x0000;
		//Notify the application that STOP is detected
		I2C_AppEventCallback(pHandle,I2C_EV_STOP);
	}

	temp3  = pHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			if(pHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pHandle);
			}
		}else
		{
			//make sure that the slave is really in transmitter mode
			if(pHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
			{
				I2C_AppEventCallback(pHandle,I2C_EV_DATA_REQ);
			}
		}
	}

	temp3  = pHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(pHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//RXNE flag is set
			if(pHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_AppEventCallback(pHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}

//IRQn32
void I2C_HandleIRQER(I2C_Handle_t *pHandle)
{
	uint32_t temp1,temp2;
	//Know the status of  ITERREN control bit in the CR2
	temp2 = (pHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);

	temp1 = (pHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//This is Bus error
		//Implement the code to clear the buss error flag
		pHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);
		//Implement the code to notify the application about the error
	    I2C_AppEventCallback(pHandle,I2C_ERROR_BERR);
	}

	temp1 = (pHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//This is arbitration lost error
		//Implement the code to clear the buss error flag
		pHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pHandle,I2C_ERROR_ARLO);
	}

	temp1 = (pHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error
		//Implement the code to clear the ACK failure error flag
		pHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pHandle,I2C_ERROR_AF);
	}

	temp1 = (pHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun
		//Implement the code to clear the ACK failure error flag
		pHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pHandle,I2C_SR1_OVR);
	}

	temp1 = (pHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error
		//Implement the code to clear the Time out error flag
		pHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pHandle,I2C_ERROR_TIMEOUT);
	}
}

void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pHandle)
{
	if(pHandle->TxLen>0)
	{
		pHandle->pI2Cx->DR = *(pHandle->pTxBuffer);//Put current data into DR
		pHandle->TxLen--;
		pHandle->pTxBuffer++;//Move to the next one
	}
}

void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pHandle)
{
	if(pHandle->RxSize == 1) {
		pHandle->pRxBuffer = pHandle->pI2Cx->DR;//Load DR into the rx buffer
		pHandle->RxLen--;
	}

	if(pHandle->RxSize > 1) {
		if(pHandle->RxSize == 2) {
			I2C_SetAcking(pHandle->pI2Cx, DISABLE);
		}
		pHandle->pRxBuffer = pHandle->pI2Cx->DR;//Load DR into the rx buffer
		pHandle->RxLen--;
		pHandle->pRxBuffer++;//Move to the next one
	}

	if(pHandle->RxLen == 0) {//All bytes read
		if (pHandle->Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pHandle->pI2Cx);
		}
		I2C_CloseReceiveData(pHandle);
		I2C_AppEventCallback(pHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pHandle->TxRxState = I2C_READY;
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxSize = 0;

	if(pHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pHandle->TxRxState = I2C_READY;
	pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
}

void I2C_SlaveSend(I2C_RegDef_t *pI2Cx,uint8_t data)
{
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceive(I2C_RegDef_t *pI2Cx)
{
    return (uint8_t) pI2C->DR;
}

void I2C_SlaveSetCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t isEnable)
{
	if (isEnable) {
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITERREN);
	} else {
		pI2Cx->CR2 &= ~((1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITERREN));
	}
}
