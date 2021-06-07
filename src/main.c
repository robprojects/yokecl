
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	
	Yoke control loader firmware
	Copyright (C) 2021 Rob Dimond (rob.dimond@gmail.com)

	Based on heavily modified FreeJoy. When the force feedback is mature, it could be backported to FreeJoy

	FreeJoy software for game device controllers
    Copyright (C) 2020  Yury Vostrenkov (yuvostrenkov@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
		
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "periphery.h"

#include "usb_hw.h"
#include "usb_lib.h"
#include "usb_pwr.h"

int enabled = 0;
int request_id_messages = 0;
int resend_keys = 0;

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The application entry point.
  *
  * @retval None
  */

uint8_t dat[64];



/* Yoke IO assignments
 *
 * PB6 - PWMA_FWD - TIM4_CH1
 * PB7 - PWMA_REV - TIM4_CH2
 * PB8 - PWMB_FWD - TIM4_CH3
 * PB9 - PMWB_REV - TIM4_CH4
 * PB10 - EN - B
 * PB11 - EN - A
 *
 * PA0 - Pot (X) - ADC12_IN0
 * PA1 - Pot (Y) - ADC12_IN1
 * PA2 - Pot (Throttle) - ADC12_IN2
 * PA3 - Pot (Mixture) - ADC12_IN3
 * PA4 - Pot (Carb heat) - ADC12_IN4
 *
 */

uint32_t old_sw = 0;
uint8_t old_keysw = 0;
uint8_t old_dome =0;
uint16_t old_adcval = 0;
uint16_t old_encoder = 0;

#define ENC_SCALE 4

uint16_t adc_buf[8];

int init_io(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIO Ports Clock Enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	// led
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);

	// motor driver
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// disable motor drivers
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_RESET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_RESET);

	// ADC for pots
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	return 0;
}

int init_adc() {
    ADC_InitTypeDef adcStructure;

    RCC_ADCCLKConfig(RCC_PCLK2_Div4); // 9MHz ADC Clock

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // ADC is DMA1 channel 1
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_BufferSize = 5;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)adc_buf;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    adcStructure.ADC_Mode = ADC_Mode_Independent;
    adcStructure.ADC_DataAlign = ADC_DataAlign_Right;
    adcStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adcStructure.ADC_ContinuousConvMode = ENABLE; //ENABLE;
    adcStructure.ADC_NbrOfChannel = 5;
    adcStructure.ADC_ScanConvMode = ENABLE;
    ADC_Init(ADC1, &adcStructure);


    //ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_28Cycles5);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	return 0;
}

int init_timers(void) {
	// setup timer
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    // usb report timer
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Prescaler = RCC_Clocks.PCLK1_Frequency/5000 - 1;
    TIM_TimeBaseInitStructure.TIM_Period = 10 - 1;                  // 1ms, 1kHz
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_SetPriority(TIM2_IRQn, 3);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM_Cmd(TIM2, ENABLE);

    // pwm timer
    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 7;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 450;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &timerInitStructure);
    TIM_Cmd(TIM4, ENABLE);

    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 0;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &outputChannelInit);
    TIM_OC2Init(TIM4, &outputChannelInit);
    TIM_OC3Init(TIM4, &outputChannelInit);
    TIM_OC4Init(TIM4, &outputChannelInit);

    return 0;
}


int main(void)
{
	SysTick_Init();
	init_io();
	init_adc();
	init_timers();

	Delay_ms(50);
	USB_HW_Init();

    TIM_SetCompare1(TIM4, 0);
    TIM_SetCompare2(TIM4, 0);

    printf("got here\n");

	for(;;);
}

