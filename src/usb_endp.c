/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "usb_hw.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_pwr.h"

#include "crc16.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

__IO uint8_t PrevXferComplete = 1;

/*
 * This is the USB HID report that the force feedback driver will send.
 */
typedef struct {
	uint8_t report; // Report ID
	uint8_t pad[3];
	int32_t centering_force[2]; // Magnitude of centering force for each axis
	int16_t offset[2]; // Offset of centre for each axis (trim)
	uint16_t pad2[2]; // 4
	/*
	 * There is a vibration synthesiser for each axis, each with 2x cos and 2x sin generators.
	 * The driver sends the amplitude and frequency.
	 */
	uint16_t s_freq[4]; // 8
	uint16_t s_amp[4]; // 8
	uint16_t c_freq[4]; // 8
	uint16_t c_amp[4]; // 8
} ff_pkt_t;

uint8_t magic_byte;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_OUT_Callback(void)
{
	uint8_t hid_buf[64];

	uint8_t repotId;

	ff_pkt_t pkt;

	/* Read received data (2 bytes) */  
	USB_SIL_Read(EP1_OUT, hid_buf);

	repotId = hid_buf[0];

	//printf("got %d %d\n", repotId, hid_buf[1]);

	switch(repotId) {
	case 2:

		memcpy(&pkt, hid_buf, sizeof(pkt));
		//printf("centering force=%08x\n", pkt.centering_force[0]);
		//printf("centering force=%08x\n", pkt.centering_force[1]);
		//printf("%d %d %d %d %d %d\n", pkt.offset[0], pkt.offset[1], pkt.s_freq[0], pkt.s_amp[0], pkt.c_freq[0], pkt.c_amp[0]);
		/*
		 * TODO This is where the force feedback packet is received... currently we don't do anything with it
		 */
		break;
	}

	memset(hid_buf, 0 ,64);
	SetEPRxStatus(ENDP1, EP_RX_VALID);

}

/*******************************************************************************
* Function Name  : EP1_IN_Callback.
* Description    : EP1 IN Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
  PrevXferComplete = 1;
}

void USB_CUSTOM_HID_Wait() {
	while(!PrevXferComplete);
}

void USB_CUSTOM_HID_SendReport(uint8_t * data, uint8_t length)
{
	if ((PrevXferComplete) && (bDeviceState == CONFIGURED))
	{
			USB_SIL_Write(EP1_IN, data, length);
			SetEPTxValid(ENDP1);
			PrevXferComplete = 0;
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

