/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/usbh_usr.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   This file includes the usb host user callbacks
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
#include "usbh_usr.h"
#include "stm32f4xx_it.h"

#include "my.h"
#include "rtc.h"

//char file_name[20];

//char fn[30]=[0,:,2,0,1,3,_,0,5,_,3,0,_,1,5,_,0,0,_,0,0,.,t,x,t];
//char fn[30]={0x30,0x3A,0x32,0x30,0x31,0x33,0x5F,0x30,0x35,0x5F,0x33,0x30,0x5F,0x31,0x35,0x5F,0x30,0x30,0x5F,0x30,0x30,0x2E,0x74,0x78,0x74};

//char fn[10]={0x30,0x3A,0x32,0x30,0x31,0x33,0x5F,0x30,0x35,0x5F};

extern u8  bufout[20];

char file_name[10]={0x20,0x65,0x72,0x72,0x6F,0x72,0x2E,0x63,0x73,0x76};

BOOL new_name_file=0; 


/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint8_t Command_index = 0;
/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */
FATFS fatfs;
FIL file;
FIL fileR;
DIR dir;
FILINFO fno;

USBH_Usr_cb_TypeDef USR_Callbacks =
{
  USBH_USR_Init,
  USBH_USR_DeInit,
  USBH_USR_DeviceAttached,
  USBH_USR_ResetDevice,
  USBH_USR_DeviceDisconnected,
  USBH_USR_OverCurrentDetected,
  USBH_USR_DeviceSpeedDetected,
  USBH_USR_Device_DescAvailable,
  USBH_USR_DeviceAddressAssigned,
  USBH_USR_Configuration_DescAvailable,
  USBH_USR_Manufacturer_String,
  USBH_USR_Product_String,
  USBH_USR_SerialNum_String,
  USBH_USR_EnumerationDone,
  USBH_USR_UserInput,
  USBH_USR_MSC_Application,
  USBH_USR_DeviceNotSupported,
  USBH_USR_UnrecoveredError
};

extern USB_OTG_CORE_HANDLE          USB_OTG_Core;
extern __IO uint8_t AudioPlayStart ;
uint8_t joystick_use = 0x00;
uint8_t lcdLineNo = 0x00;
extern __IO uint8_t RepeatState ;
extern __IO uint8_t LED_Toggle;
static uint8_t USBH_USR_ApplicationState = USH_USR_FS_INIT;
//extern __IO uint32_t WaveDataLength ;
extern __IO uint16_t Time_Rec_Base;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  USBH_USR_Init
  * @param  None
  * @retval None
  */
void USBH_USR_Init(void)
{
}

/**
  * @brief  USBH_USR_DeviceAttached
  * @param  None
  * @retval None
  */
void USBH_USR_DeviceAttached(void)
{
		TDateTime DT;
		u8 tmp;
 /*   
	  rtc_Get(&DT);    
  
//	file_name[0]=0x32;
//	file_name[1]=0x30;
	
	file_name[0]=(uint8_t)(DT.Year/10)+(uint8_t)0x30;
	file_name[1]=(uint8_t)(DT.Year%10)+(uint8_t)0x30;

//	file_name[2]=95;
	
	file_name[2]=(uint8_t)(DT.Month/10)+(uint8_t)0x30;
	file_name[3]=(uint8_t)(DT.Month%10)+(uint8_t)0x30;

//	file_name[5]=95;
	
	file_name[4]=(uint8_t)(DT.Day/10)+(uint8_t)0x30;
	file_name[5]=(uint8_t)(DT.Day%10)+(uint8_t)0x30;
	*/
/*
	file_name[10]=95;
	
	tmp=DT.Hours/10;
	file_name[11]=(uint8_t)(tmp)+(uint8_t)0x30;
	tmp%=10;
	file_name[12]=(uint8_t)(tmp)+(uint8_t)0x30;

	file_name[13]=95;

	tmp=DT.Minutes/10;
	file_name[14]=(uint8_t)(tmp)+(uint8_t)0x30;
	tmp%=10;
	file_name[15]=(uint8_t)(tmp)+(uint8_t)0x30;
*/
/*
	file_name[0]=(uint8_t)(DT.Year/10)+(uint8_t)0x30;
	file_name[1]=(uint8_t)(DT.Year%10)+(uint8_t)0x30;

	file_name[2]=(uint8_t)(DT.Month/10)+(uint8_t)0x30;
	file_name[3]=(uint8_t)(DT.Month%10)+(uint8_t)0x30;

	file_name[4]=(uint8_t)(DT.Day/10)+(uint8_t)0x30;
	file_name[5]=(uint8_t)(DT.Day%10)+(uint8_t)0x30;
	*/
	
	file_name[0]=(uint8_t)(bufout[6]/10)+(uint8_t)0x30;
	file_name[1]=(uint8_t)(bufout[6]%10)+(uint8_t)0x30;

	file_name[2]=(uint8_t)(bufout[5]/10)+(uint8_t)0x30;
	file_name[3]=(uint8_t)(bufout[5]%10)+(uint8_t)0x30;

	file_name[4]=(uint8_t)(bufout[4]/10)+(uint8_t)0x30;
	file_name[5]=(uint8_t)(bufout[4]%10)+(uint8_t)0x30;
	
// .txt
	file_name[6]=0x2E;
	file_name[7]=0x63;
	file_name[8]=0x73;
	file_name[9]=0x76;

	sost_flesh=1;
	PORT_ZAP_EN->BSRRL = PIN_ZAP_EN;  // on  PORT_ZAP_EN
	PORT_ZAP_DIS->BSRRH = PIN_ZAP_DIS;  // off  PORT_ZAP_DIS
	

/* Red LED off when device attached */
  STM_EVAL_LEDOff(LED5);
  /* Green LED on */
  STM_EVAL_LEDOn(LED6);
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
}

/**
  * @brief  USBH_USR_UnrecoveredError
  * @param  None
  * @retval None
  */
void USBH_USR_UnrecoveredError (void)
{
}

/**
  * @brief  USBH_DisconnectEvent
  *         Device disconnect event
  * @param  None
  * @retval Staus
  */
void USBH_USR_DeviceDisconnected (void)
{
	
		sost_flesh=0;	
		PORT_ZAP_EN->BSRRH = PIN_ZAP_EN;  // off  PORT_ZAP_EN
		PORT_ZAP_DIS->BSRRL = PIN_ZAP_DIS;  // on  PORT_ZAP_DIS
	
  /* Red Led on if the USB Key is removed */
  STM_EVAL_LEDOff(LED6);
	STM_EVAL_LEDOff(LED5);
	STM_EVAL_LEDOff(LED4);
  /* Disable the Timer */
  TIM_ITConfig(TIM4, TIM_IT_CC1 , DISABLE);

}

/**
  * @brief  USBH_USR_ResetUSBDevice
  * @param  None
  * @retval None
  */
void USBH_USR_ResetDevice(void)
{
  /* callback for USB-Reset */
}


/**
  * @brief  USBH_USR_DeviceSpeedDetected
  *         Displays the message on LCD for device speed
  * @param  Device speed:
  * @retval None
  */
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
}

/**
  * @brief  USBH_USR_Device_DescAvailable
  * @param  device descriptor
  * @retval None
  */
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
  /* callback for device descriptor */
}

/**
  * @brief  USBH_USR_DeviceAddressAssigned
  *         USB device is successfully assigned the Address
  * @param  None
  * @retval None
  */
void USBH_USR_DeviceAddressAssigned(void)
{
  /* callback for device successfully assigned the Address */
}

/**
  * @brief  USBH_USR_Conf_Desc
  * @param  Configuration descriptor
  * @retval None
  */
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
    USBH_InterfaceDesc_TypeDef *itfDesc,
    USBH_EpDesc_TypeDef *epDesc)
{
  /* callback for configuration descriptor */
}

/**
  * @brief  USBH_USR_Manufacturer_String
  * @param  Manufacturer String
  * @retval None
  */
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
  /* callback for  Manufacturer String */
}

/**
  * @brief  USBH_USR_Product_String
  * @param  Product String
  * @retval None
  */
void USBH_USR_Product_String(void *ProductString)
{
  /* callback for Product String */
}

/**
  * @brief  USBH_USR_SerialNum_String
  * @param  SerialNum_String
  * @retval None
  */
void USBH_USR_SerialNum_String(void *SerialNumString)
{
  /* callback for SerialNum_String */
}

/**
  * @brief  EnumerationDone 
  *         User response request is displayed to ask application jump to class
  * @param  None
  * @retval None
  */
void USBH_USR_EnumerationDone(void)
{
  /* 0.5 seconds delay */
  USB_OTG_BSP_mDelay(500);
  
  USBH_USR_MSC_Application();
} 

/**
  * @brief  USBH_USR_DeviceNotSupported
  *         Device is not supported
  * @param  None
  * @retval None
  */
void USBH_USR_DeviceNotSupported(void)
{
		sost_flesh=0;	
		PORT_ZAP_EN->BSRRH = PIN_ZAP_EN;  // off  PORT_ZAP_EN
		PORT_ZAP_DIS->BSRRL = PIN_ZAP_DIS;  // on  PORT_ZAP_DIS
}


/**
  * @brief  USBH_USR_UserInput
  *         User Action for application state entry
  * @param  None
  * @retval USBH_USR_Status : User response for key button
  */
USBH_USR_Status USBH_USR_UserInput(void)
{
  /* callback for Key botton: set by software in this case */
  return USBH_USR_RESP_OK;
}

/**
  * @brief  USBH_USR_OverCurrentDetected
  *         Over Current Detected on VBUS
  * @param  None
  * @retval None
  */
void USBH_USR_OverCurrentDetected (void)
{
}

/**
  * @brief  USBH_USR_MSC_Application
  * @param  None
  * @retval Staus
  */
int USBH_USR_MSC_Application(void)
{

  switch (USBH_USR_ApplicationState)
  {
    case USH_USR_FS_INIT:

      // Initialises the File System*/
      if (f_mount( 0, &fatfs ) != FR_OK )
      {
        // efs initialisation fails
				sost_flesh=0;	
				PORT_ZAP_EN->BSRRH = PIN_ZAP_EN;  // off  PORT_ZAP_EN
				PORT_ZAP_DIS->BSRRL = PIN_ZAP_DIS;  // on  PORT_ZAP_DIS
        return(-1);
      }
      
      // Flash Disk is write protected //
      if (USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
      {
				sost_flesh=0;	
				PORT_ZAP_EN->BSRRH = PIN_ZAP_EN;  // off  PORT_ZAP_EN
				PORT_ZAP_DIS->BSRRL = PIN_ZAP_DIS;  // on  PORT_ZAP_DIS
        while(1)
        {
          // Red LED On //
          STM_EVAL_LEDOn(LED5);
        }
      }
      // Go to menu 
      USBH_USR_ApplicationState = USH_USR_AUDIO;
      break;

    case USH_USR_AUDIO:

      // Go to Audio menu 
      COMMAND_AudioExecuteApplication();

      // Set user initialization flag //
      USBH_USR_ApplicationState = USH_USR_FS_INIT;
      break;

    default:
      break;
  }
  return(0);
}



void dec_to_chr(u16 chislo,uint8_t* buf)
{
	// BYTE *wbuff = buf;
	u16 tmp=0, tk_n=0;
		
	if ((chislo&0x8000)==1)
	{
		tk_n=1;
		chislo=chislo&0x7FFF;
	}
	
	//tmp=chislo/10000;
	if (tk_n==1)
	{
		*buf=(uint8_t)0x30;
		buf++;
	}
		
	*buf=(uint8_t)(chislo/10000)+(uint8_t)0x30;
	chislo%=10000;
	buf++;
	*buf=((uint8_t)(chislo/1000)+(uint8_t)0x30);
	chislo%=1000;
	buf++;
	*buf=((uint8_t)(chislo/100)+(uint8_t)0x30);
	chislo%=100;
	buf++;
	*buf=((uint8_t)(chislo/10)+(uint8_t)0x30);
	buf++;
	if (tk_n==0)
	{
		*buf=0x2C;
		chislo%=10;
		buf++;
	}
	*buf=((uint8_t)(chislo)+(uint8_t)0x30);
			

}

/**
  * @brief  COMMAND_AudioExecuteApplication
  * @param  None
  * @retval None
  */
void COMMAND_AudioExecuteApplication(void)
{
  /* Execute the command switch the command index */
	
//	WaveRecorderUpdate();
	/*
  switch (Command_index)
  {
  // Start Playing from USB Flash memory 
  case CMD_PLAY:
    if (RepeatState == 0)
//      WavePlayerStart();
    break;
    // Start Recording in USB Flash memory  
  case CMD_RECORD:
    RepeatState = 0;
    WaveRecorderUpdate();
    break;  
  default:
    break;
  }
	*/
//	minute=12345;
//	if (minute!=pred_minute)
if (buffering)
	{
		TDateTime DT;
		u8 tmp=0;
    
//	  rtc_Get(&DT);     	
//		pred_minute=minute;
//		minute=12345;
		
	//		dec_to_chr(minute,(uint8_t*) &Buf_zap[0]);
		/*
	Buf_zap[0]=0x32;
	Buf_zap[1]=0x30;
	
	tmp=DT.Year/10;
	Buf_zap[2]=(uint8_t)(tmp)+(uint8_t)0x30;
	tmp%=10;
	Buf_zap[3]=(uint8_t)(tmp)+(uint8_t)0x30;

	Buf_zap[4]=95;
	
	tmp=DT.Month/10;
	Buf_zap[5]=(uint8_t)(tmp)+(uint8_t)0x30;
	tmp%=10;
	Buf_zap[6]=(uint8_t)(tmp)+(uint8_t)0x30;

	Buf_zap[7]=95;
	
	tmp=DT.Day/10;
	Buf_zap[8]=(uint8_t)(tmp)+(uint8_t)0x30;
	tmp%=10;
	Buf_zap[9]=(uint8_t)(tmp)+(uint8_t)0x30;

	Buf_zap[10]=95;
	*/


	Buf_zap[0]=(uint8_t)(DT_zap.Hours/10)+(uint8_t)0x30;
	Buf_zap[1]=(uint8_t)(DT_zap.Hours%10)+(uint8_t)0x30;
	Buf_zap[2]=0x3A; // :
	Buf_zap[3]=(uint8_t)(DT_zap.Minutes/10)+(uint8_t)0x30;
	Buf_zap[4]=(uint8_t)(DT_zap.Minutes%10)+(uint8_t)0x30;
	Buf_zap[5]=0x3A;
	Buf_zap[6]=(uint8_t)(DT_zap.Seconds/10)+(uint8_t)0x30;
	Buf_zap[7]=(uint8_t)(DT_zap.Seconds%10)+(uint8_t)0x30;
	
/*
	Buf_zap[0]=(uint8_t)(bufout[2]/10)+(uint8_t)0x30;
	Buf_zap[1]=(uint8_t)(bufout[2]%10)+(uint8_t)0x30;
	Buf_zap[2]=0x3A; // :
	Buf_zap[3]=(uint8_t)(bufout[1]/10)+(uint8_t)0x30;
	Buf_zap[4]=(uint8_t)(bufout[1]%10)+(uint8_t)0x30;
	Buf_zap[5]=0x3A;
	Buf_zap[6]=(uint8_t)(bufout[0]/10)+(uint8_t)0x30;
	Buf_zap[7]=(uint8_t)(bufout[0]%10)+(uint8_t)0x30;	
	*/
		//	Buf_zap[8]=0x3B; //	;
		//	Buf_zap[9]=0x0D; //	enter
			
	/*	
			dec_to_chr(time_label,(uint8_t*) &Buf_zap[6]);
			
		  Buf_zap[11]=0x20;

			sm=12;
			
			dec_to_chr(time_label,(uint8_t*) &Buf_zap[20]);
			Buf_zap[25]=0x3B;
			
			*/
			
			sm=8;

			cnt=0;
			kol_zap=0;
		
			

		if (number_buff)	
			while (kol_zap<600)
      {			
			//	Buf_adc_zap1[kol_zap]=kol_zap;
				Buf_zap[sm+kol_simb_in_stroka*kol_zap]=0x3B; //	;
			 dec_to_chr(Buf_adc_zap1[kol_zap],(uint8_t*) &Buf_zap[sm+kol_simb_in_stroka*kol_zap+1]);			
				kol_zap++;			
				Buf_zap[sm+kol_simb_in_stroka*kol_zap-1]=0x0D;	// enter
			}
		else
			while (kol_zap<600)
      {			
				while (kol_zap<600)
				{			
				//	Buf_adc_zap2[kol_zap]=kol_zap+50000;
					Buf_zap[sm+kol_simb_in_stroka*kol_zap]=0x3B; //	;
				 dec_to_chr(Buf_adc_zap2[kol_zap],(uint8_t*) &Buf_zap[sm+kol_simb_in_stroka*kol_zap+1]);			
					kol_zap++;			
					Buf_zap[sm+kol_simb_in_stroka*kol_zap-1]=0x0D;	// enter
				}		  	
			}		
			
//			Buf_zap[sm+6*kol_zap]=13;
			
	//		por=0;
			
	//		f_unlink (REC_WAVE_NAME);
	
	//		f_open(&file, REC_WAVE_NAME, FA_CREATE_ALWAYS | FA_WRITE);
	//		f_open(&file, REC_WAVE_NAME, FA_WRITE);
			
		/*
			STM_EVAL_LEDOff(LED3);
			STM_EVAL_LEDOff(LED4);
			STM_EVAL_LEDOff(LED5);
			STM_EVAL_LEDOff(LED6);
		*/	
				/* Set ON Red LED */ 
			/*
				while ((HCD_IsDeviceConnected(&USB_OTG_Core) != 1))
				{
					STM_EVAL_LEDOff(LED5);
	//				Delay	(1000);		
				}
			*/	
				if (HCD_IsDeviceConnected(&USB_OTG_Core) == 1)
				{
					
		//			if (!file_cr)
					{
					if (f_open(&file,  (const XCHAR *)&file_name, FA_WRITE)== FR_OK)	
				//		if (f_open(&file, (const XCHAR *)&file_name, FA_CREATE_ALWAYS)== FR_OK)
						{
							STM_EVAL_LEDOn(LED4);
							f_lseek(&file, (DWORD)(file.fsize));
							f_write (&file, (uint8_t*)Buf_zap, sm+kol_simb_in_stroka*kol_zap, (void *)&bytesWritten); 				
							f_close (&file);

					//		if 	(DT.Minutes==0)
					//			if (DT.Hours==0)
							//		if (file_name[5]!=(DT.Day%10+0x30))
									if (file_name[5]!=(bufout[4]%10+0x30))
									{
										/*
										file_name[0]=(uint8_t)(DT.Year/10)+(uint8_t)0x30;
										file_name[1]=(uint8_t)(DT.Year%10)+(uint8_t)0x30;
									
										file_name[2]=(uint8_t)(DT.Month/10)+(uint8_t)0x30;
										file_name[3]=(uint8_t)(DT.Month%10)+(uint8_t)0x30;

										file_name[4]=(uint8_t)(DT.Day/10)+(uint8_t)0x30;
										file_name[5]=(uint8_t)(DT.Day%10)+(uint8_t)0x30;
										*/
										file_name[0]=(uint8_t)(bufout[6]/10)+(uint8_t)0x30;
										file_name[1]=(uint8_t)(bufout[6]%10)+(uint8_t)0x30;
									
										file_name[2]=(uint8_t)(bufout[5]/10)+(uint8_t)0x30;
										file_name[3]=(uint8_t)(bufout[5]%10)+(uint8_t)0x30;

										file_name[4]=(uint8_t)(bufout[4]/10)+(uint8_t)0x30;
										file_name[5]=(uint8_t)(bufout[4]%10)+(uint8_t)0x30;
									}							
						STM_EVAL_LEDOff(LED4);							
						file_cr=1;
						sost_flesh=1;	
						PORT_ZAP_EN->BSRRL = PIN_ZAP_EN;  // on  PORT_ZAP_EN
						PORT_ZAP_DIS->BSRRH = PIN_ZAP_DIS;  // off  PORT_ZAP_DIS
						}
						else
						{
							if (f_open(&file, (const XCHAR *)&file_name, FA_CREATE_ALWAYS)== FR_OK)
					//		if (f_open(&file,  (const XCHAR *)&file_name, FA_WRITE)== FR_OK)
							{
								
								STM_EVAL_LEDOn(LED4);
								f_lseek(&file,(DWORD) file.fsize);				
								f_write (&file, (uint8_t*)Buf_zap, sm+kol_simb_in_stroka*kol_zap, (void *)&bytesWritten); 				
								f_close (&file);
								
								//		if (file_name[5]!=(DT.Day%10+0x30))
										if (file_name[5]!=(bufout[4]%10+0x30))
										{
										file_name[0]=(uint8_t)(bufout[6]/10)+(uint8_t)0x30;
										file_name[1]=(uint8_t)(bufout[6]%10)+(uint8_t)0x30;
									
										file_name[2]=(uint8_t)(bufout[5]/10)+(uint8_t)0x30;
										file_name[3]=(uint8_t)(bufout[5]%10)+(uint8_t)0x30;

										file_name[4]=(uint8_t)(bufout[4]/10)+(uint8_t)0x30;
										file_name[5]=(uint8_t)(bufout[4]%10)+(uint8_t)0x30;
											/*
										file_name[0]=(uint8_t)(DT.Year/10)+(uint8_t)0x30;
										file_name[1]=(uint8_t)(DT.Year%10)+(uint8_t)0x30;
									
										file_name[2]=(uint8_t)(DT.Month/10)+(uint8_t)0x30;
										file_name[3]=(uint8_t)(DT.Month%10)+(uint8_t)0x30;

										file_name[4]=(uint8_t)(DT.Day/10)+(uint8_t)0x30;
										file_name[5]=(uint8_t)(DT.Day%10)+(uint8_t)0x30;
											*/
										}
								
								STM_EVAL_LEDOff(LED4);
								sost_flesh=1;	
								PORT_ZAP_EN->BSRRL = PIN_ZAP_EN;  // on  PORT_ZAP_EN
								PORT_ZAP_DIS->BSRRH = PIN_ZAP_DIS;  // off  PORT_ZAP_DIS
							}
							else
							{
								STM_EVAL_LEDOn(LED5);
								sost_flesh=0;	
								PORT_ZAP_EN->BSRRH = PIN_ZAP_EN;  // off  PORT_ZAP_EN
								PORT_ZAP_DIS->BSRRL = PIN_ZAP_DIS;  // on  PORT_ZAP_DIS
							}								
						}							
					}
				}
				else
				{
					file_cr=0;
					sost_flesh=0;	
					PORT_ZAP_EN->BSRRH = PIN_ZAP_EN;  // off  PORT_ZAP_EN
					PORT_ZAP_DIS->BSRRL = PIN_ZAP_DIS;  // on  PORT_ZAP_DIS
				}
				
			buffering=0;
		}
}

/**
  * @brief  USBH_USR_DeInit
  *         Deint User state and associated variables
  * @param  None
  * @retval None
  */
void USBH_USR_DeInit(void)
{
  USBH_USR_ApplicationState = USH_USR_FS_INIT;
}

/**
  * @}
  */



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
