/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

#include "usb_istr.h"
#include "usb_lib.h"
#include "periphery.h"
#include "usb_desc.h"

typedef int16_t analog_data_t;

//extern uint8_t magic_byte;



/*
typedef struct
{
//        uint8_t                                 dummy;          // alighning
        uint8_t                                 id;
//        analog_data_t                           raw_axis_data[MAX_AXIS_NUM];
//        uint8_t                                 raw_button_data[9];
//        uint8_t                                 hift_button_data;
        uint8_t                                 button_data[MAX_BUTTONS_NUM/8];
        analog_data_t                           axis_data[MAX_AXIS_NUM];
//        uint8_t                                 pov_data[MAX_POVS_NUM];


} joy_report_t __attribute__((packed));
*/

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

#if 1
void SysTick_Handler(void)
{

	if (TimingDelay != 0x00)										
	{
		TimingDelay--;
	}
}
#endif

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
uint64_t ticks = 0;

uint64_t last_tick = 0;
uint64_t print_tick = 0;


//FIXME!
typedef int32_t fx_t;

#define FX_FRAC 16
#define fx_fx2int(a) ((int)((a)>>FX_FRAC))

extern fx_t axis_filt[5];

void TIM2_IRQHandler(void) {
	//static uint8_t btn_num = 0;
	//uint8_t physical_buttons_data[MAX_BUTTONS_NUM];
	//joy_report_t joy_report;
	uint8_t joy_report[1 + MAX_BUTTONS_NUM/8 + MAX_AXIS_NUM*2];

	int16_t axis[MAX_AXIS_NUM];

	// axis range is -32767 to 32767

	if (TIM_GetITStatus(TIM2, TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		ticks++;

		// compute a force feedback cycle every 1us
		ff_cycle();

		if (ticks - last_tick >= 50) {
			// every 50us send a USB HID joystick report
			last_tick = ticks;
			int i;
			for (i=0; i<MAX_AXIS_NUM; i++) {
				if (i < 5) {
					axis[i] = fx_fx2int(axis_filt[i])<<5;
				} else {
					axis[i] = 0;
				}
			}



			//for (i=0; i<MAX_POVS_NUM; i++)
			//	joy_report.pov_data[i] = 0;

			/*joy_report.raw_button_data[0] = btn_num;
			for (uint8_t i=0; i<64; i++) {
				joy_report.raw_button_data[1 + ((i & 0xF8)>>3)] &= ~(1 << (i & 0x07));
				joy_report.raw_button_data[1 + ((i & 0xF8)>>3)] |= physical_buttons_data[btn_num+i] << (i & 0x07);
			}*/
			//btn_num += 64;
			//btn_num = btn_num & 0x7F;

			joy_report[0] = REPORT_ID_JOY;
			//joy_report[2] = (ticks&0x800)? 0x00:0x04;
			//joy_report[1] = 0x00;
			int pos = 1;
			for (i=0; i<MAX_AXIS_NUM; i++) {
				joy_report[pos++] = axis[i]&0xff;
				joy_report[pos++] = axis[i]>>8;
				//pos+=2;
			}

			// flaps switch
			int buttons = 0;
			buttons |= (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12));
			buttons |= (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)) << 1;

			// carb heat
			if (axis_filt[4] > 0) {
				buttons |= 1<<2;
			} else {
				buttons |= 1<<3;
			}

			joy_report[pos++] = buttons; //(ticks&0x800)?0x00 : 0x01;
			joy_report[pos] = 0x00;
			USB_CUSTOM_HID_SendReport((uint8_t *)joy_report, sizeof(joy_report)/* - sizeof(joy_report.dummy)*/);
		}
#if 0
		if ((ticks - print_tick) > 1000) {
			print_tick = ticks;
			printf("motor %d\n", fx_fx2int(axis_filt[2])/2);
		}
#endif
	}
}

/**
* @brief This function handles USB low priority or CAN RX0 interrupts.
*/
#if 1
void USB_LP_CAN1_RX0_IRQHandler(void)
{	
	USB_Istr();
}
#endif

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
