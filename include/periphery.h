/**
  ******************************************************************************
  * @file           : periphery.h
  * @brief          : Header for periphery.c file.
  *                   This file contains the common defines of the periphery.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PERIPHERY_H__
#define __PERIPHERY_H__

#include "stm32f10x.h"
#include "stm32f10x_conf.h"


extern volatile uint64_t Ticks;
extern volatile uint32_t TimingDelay;



void SysTick_Init(void);

uint64_t GetTick(void);

void Delay_ms(__IO uint32_t nTime);
void Delay_us(__IO uint32_t nTime);




#endif 	/* __PERIPHERY_H__ */
