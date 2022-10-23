/*
 * i2c.c
 *
 *  Created on: 23 Eki 2022
 *      Author: Veli Ertunç
 */

#include "i2c.h"

void SetPeripheralClock(I2CRegDef_t *pI2Cx,uint8_t isEnable)
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
