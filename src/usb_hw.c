#include "usb_hw.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define         ID1          (0x1FFFF7E8)
#define         ID2          (0x1FFFF7EC)
#define         ID3          (0x1FFFF7F0)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

ErrorStatus HSEStartUpStatus;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
static void AsciiToUnicode (uint8_t * pbuf_in , uint8_t *pbuf_out , uint8_t len);
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;  
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32xxx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32xxx.c file
     */ 
  
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD) || defined(STM32F37X) || defined(STM32F303xC) || defined(STM32F303xE)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

#else /* defined(STM32F10X_HD) || defined(STM32F10X_MD) defined(STM32F10X_XL)*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
#endif
  
  /********************************************/
  /*  Configure USB DM/DP pins                */
  /********************************************/
  
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)
  
	  /* Enable all GPIOs Clock*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ALLGPIO, ENABLE);
	
	  /* Configure USB DM/DP pin. This is optional, and maintained only for user guidance.
  For the STM32L products there is no need to configure the PA12/PA11 pins couple 
  as Alternate Function */ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  
#elif defined(STM32F10X_HD) || defined(STM32F10X_MD)  || defined(STM32F10X_XL)

/* Enable all GPIOs Clock*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIOA->ODR &= ~GPIO_Pin_12;			// Enumeration
	Delay_ms(5); //delay
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
  
#else /* defined(STM32F37X) || defined(STM32F303xC) */
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  /*SET PA11,12 for USB: USB_DM,DP*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_14);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_14);
  
#endif 
  
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  
#else
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config.
* Description    : Configures the USB interrupts.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
  
  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
  
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_FS_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#elif defined(STM32F37X)
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#else
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif /* STM32L1XX_XD */
  
}


/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;
  
  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;
  
  Device_Serial0 += Device_Serial2;
  
  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &CustomHID_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &CustomHID_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : Get_ProductStr.
* Description    : Create the product string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_ProductStr(void)
{
	//dev_config_t tmp;
	
	//DevConfigGet(&tmp);
	char device_name[] = "Teensy";
  AsciiToUnicode((uint8_t *) device_name, &CustomHID_StringProduct[2], sizeof(device_name));
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/*******************************************************************************
* Function Name  : AsciiToUnicode
* Description    : Convert ASCII string value into unicode array.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void AsciiToUnicode (uint8_t * pbuf_in , uint8_t *pbuf_out , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {    
    pbuf_out[2* idx] = pbuf_in[idx];
    pbuf_out[2* idx + 1] = 0;
  }
}

/*******************************************************************************
* Function Name  : Get_VidPid.
* Description    : Change VID and PID.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_VidPid(void)
{
	//dev_config_t tmp;
	
	//DevConfigGet(&tmp);
	uint16_t vid = 0x16c0, pid = 0x0488;
	
  CustomHID_DeviceDescriptor[8] = LOBYTE(vid);
  CustomHID_DeviceDescriptor[9] = HIBYTE(vid);
  CustomHID_DeviceDescriptor[10] = LOBYTE(pid);
  CustomHID_DeviceDescriptor[11] = HIBYTE(pid);
}

/*******************************************************************************
* Function Name  : Get_ReportDesc.
* Description    : Change VID and PID.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_ReportDesc(void)
{
	//app_config_t tmp_app_config;
	
	//AppConfigGet(&tmp_app_config);
/*
	CustomHID_ReportDescriptor[104] = tmp_app_config.buttons_cnt ? tmp_app_config.buttons_cnt : 1;
	CustomHID_ReportDescriptor[112] = tmp_app_config.buttons_cnt ? ((tmp_app_config.buttons_cnt - 1)/8 + 1) * 8 : 0;
	
	CustomHID_ReportDescriptor[41] = (tmp_app_config.axes & 0x01) ? 0x30 : 0x04;
	CustomHID_ReportDescriptor[43] = (tmp_app_config.axes & 0x02) ? 0x31 : 0x04;
	CustomHID_ReportDescriptor[45] = (tmp_app_config.axes & 0x04) ? 0x32 : 0x04;
	CustomHID_ReportDescriptor[47] = (tmp_app_config.axes & 0x08) ? 0x33 : 0x04;
	CustomHID_ReportDescriptor[49] = (tmp_app_config.axes & 0x10) ? 0x34 : 0x04;
	CustomHID_ReportDescriptor[51] = (tmp_app_config.axes & 0x20) ? 0x35 : 0x04;
	CustomHID_ReportDescriptor[53] = (tmp_app_config.axes & 0x40) ? 0x36 : 0x04;
	CustomHID_ReportDescriptor[55] = (tmp_app_config.axes & 0x80) ? 0x36 : 0x04;
	
	CustomHID_ReportDescriptor[69] = (tmp_app_config.povs & 0x01) ? 0x39 : 0x00;
	CustomHID_ReportDescriptor[88] = (tmp_app_config.povs & 0x02) ? 0x39 : 0x00;
	CustomHID_ReportDescriptor[92] = (tmp_app_config.povs & 0x04) ? 0x39 : 0x00;
	CustomHID_ReportDescriptor[96] = (tmp_app_config.povs & 0x08) ? 0x39 : 0x00;
	*/
}

void USB_HW_Init(void)
{
	Set_System();

  USB_Interrupts_Config();

  Set_USBClock();

  USB_Init();
	
}

void USB_HW_DeInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;  

	// Disable USB Clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIOA->ODR &= ~(GPIO_Pin_11 | GPIO_Pin_12);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

