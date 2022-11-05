/*
 * rcc.h
 *
 *  Created on: 15 Oct 2022
 *      Author: Veli Ertunç
 */

#ifndef RCC_H_
#define RCC_H_
#include "baseaddr.h"

/*
 * RCC register definition
 */
typedef volatile struct
{
  uint32_t CR;            /* Clock control register,     								Address offset: 0x00 */
  uint32_t PLLCFGR;       /* PLL configuration register,     							Address offset: 0x04 */
  uint32_t CFGR;          /* Clock configuration register,     							Address offset: 0x08 */
  uint32_t CIR;           /* Clock interrupt register,     								Address offset: 0x0C */
  uint32_t AHB1RSTR;      /* AHB1 peripheral reset register,     						Address offset: 0x10 */
  uint32_t AHB2RSTR;      /* AHB2 peripheral reset register,     						Address offset: 0x14 */
  uint32_t AHB3RSTR;      /* AHB3 peripheral reset register,     						Address offset: 0x18 */
  uint32_t RESERVED0;     /* Reserved, 0x1C                                                       */
  uint32_t APB1RSTR;      /* APB1 peripheral reset register,     						Address offset: 0x20 */
  uint32_t APB2RSTR;      /* APB2 peripheral reset register,     						Address offset: 0x24 */
  uint32_t RESERVED1[2];  /* Reserved, 0x28-0x2C                                                  */
  uint32_t AHB1ENR;       /* AHB1 peripheral clock register,     						Address offset: 0x30 */
  uint32_t AHB2ENR;       /* AHB2 peripheral clock register,     						Address offset: 0x34 */
  uint32_t AHB3ENR;       /* AHB3 peripheral clock register,     						Address offset: 0x38 */
  uint32_t RESERVED2;     /* Reserved, 0x3C                                                       */
  uint32_t APB1ENR;       /* APB1 peripheral clock register,     						Address offset: 0x40 */
  uint32_t APB2ENR;       /* APB2 peripheral clock register,     						Address offset: 0x44 */
  uint32_t RESERVED3[2];  /* Reserved, 0x48-0x4C                                                  */
  uint32_t AHB1LPENR;     /* AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50 */
  uint32_t AHB2LPENR;     /* AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54 */
  uint32_t AHB3LPENR;     /* AHB3 peripheral clock enable in low power mode register,	Address offset: 0x58 */
  uint32_t RESERVED4;     /* Reserved, 0x5C                                                       */
  uint32_t APB1LPENR;     /* APB1 peripheral clock enable in low power mode register,   Address offset: 0x60 */
  uint32_t APB2LPENR;     /* APB2 peripheral clock enable in low power mode register,	Address offset: 0x64 */
  uint32_t RESERVED5[2];  /* Reserved, 0x68-0x6C                                                  */
  uint32_t BDCR;          /* Backup domain control register,     						Address offset: 0x70 */
  uint32_t CSR;           /* Clock control & status register,     						Address offset: 0x74 */
  uint32_t RESERVED6[2];  /* Reserved, 0x78-0x7C                                                  */
  uint32_t SSCGR;         /* Spread spectrum clock generation register,     			Address offset: 0x80 */
  uint32_t PLLI2SCFGR;    /* PLLI2S configuration register,     						Address offset: 0x84 */
  uint32_t PLLSAICFGR;    /* PLL configuration register,     							Address offset: 0x88 */
  uint32_t DCKCFGR;       /* Dedicated Clock Configuration Register,     				Address offset: 0x8C */
} RCC_RegDef_t;

#define RCC				((RCC_RegDef_t*)RCC_BASE)


uint32_t RCC_GetPCLK1Value(void);

uint32_t RCC_GetPCLK2Value(void);

uint32_t  RCC_GetPLLOutputClock(void);

#endif /* RCC_H_ */
