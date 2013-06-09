/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
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
#include "main.h"

#include "my.h"
#include "rtc.h"

TDateTime DT1;
		

		//		u8 tmp1=0;
		//		u8 tmp2=0;
		//		u8 tmp3=0;



/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint8_t PauseResumeStatus = 2, Count = 0, LED_Toggle = 0;
uint16_t capture = 0;
extern __IO uint16_t CCR_Val;
extern __IO uint8_t RepeatState; // , AudioPlayStart;
//extern uint8_t Buffer[];

#if defined MEDIA_USB_KEY
__IO uint16_t Time_Rec_Base = 0;
 extern USB_OTG_CORE_HANDLE          USB_OTG_Core;
 extern USBH_HOST                    USB_Host;
 extern FIL file;
 extern __IO uint8_t Data_Status;
// extern __IO uint32_t XferCplt ;
 extern __IO uint8_t Command_index;
#endif /* MEDIA_USB_KEY */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/


/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
  /* This interrupt is generated when HSE clock fails */

  if (RCC_GetITStatus(RCC_IT_CSS) != RESET)
  {
    /* At this stage: HSE, PLL are disabled (but no change on PLL config) and HSI
       is selected as system clock source */

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Enable HSE Ready and PLL Ready interrupts */
    RCC_ITConfig(RCC_IT_HSERDY | RCC_IT_PLLRDY, ENABLE);

    /* Clear Clock Security System interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_CSS);

    /* Once HSE clock recover, the HSERDY interrupt is generated and in the RCC ISR
       routine the system clock will be reconfigured to its previous state (before
       HSE clock failure) */
  }
}


/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

u16 fiz_vel(u16 kod, u8 numb)
{
	
//	tab_kal
	
return kod;
}


u16 moving_average(u16 kod, u8 numb)
{
return kod;
}

u8 test_rele(u16 kod, u8 numb)
{
return 0;
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	extern __IO uint16_t ADC3ConvertedValue;
	extern u32 tick;
	
	
	
	// раз в 10 мс
	// находим среднее значение
		summa[0]+=ADC3ConvertedValue;

	 
//		summa[0]+=100;
		kol_average++;
	
	if (kol_average==10)
	{
	
		average[0]=summa[0]/kol_average;
		kol_average=0;
		summa[0]=0;
		
		
		// раз в 100 мс
		// вычисляем физическую виличину 
	
		fz[0]=fiz_vel(average[0],0);
	
		// проверяем реле на срабатывание
		test_rele(fz[0], 0);	
	
	// находим среднее значение по скользящей средней
		fz_average[0]=moving_average(fz[0],0);
		
		if (fz_average[0]<max[0])
			max[0]=fz_average[0];
		
		if (number_buff)
			Buf_adc_zap2[por++]=por; //fz_average[0];			
		else
			Buf_adc_zap1[por++]=por; //fz_average[0];
		
		if (por==999)
			por=999;
	
	del++;
	if (del==10)
	{
		
		del=0;
		tick++;
		time_label=tick;

		
		rtc_Get(&DT1);

		
		if (DT1.Seconds==0)
		{
			number_buff^=1;
			DT_zap=DT1;
			
			buffering=1;
			por=0;
		}
			
	 if (tick%2==0)
	 {
		 STM_EVAL_LEDOn(LED3);		 
	 }
	 else
	 {
		 STM_EVAL_LEDOff(LED3);	 
	 }
	 
 indicate_lin(0,(u16)fz_average[0], 4096);
	  // indicate(0,0);
 indicate(1,(u16)(fz_average[0]/10));
		 
	 }		 
		
	if ((tick%60)==0)
	{
		minute++;
	}
	}
	
	
	if (new_komand)
	{
		u16 tmp=0;
	
		
		if ((RxBuffer[2]=='w')&(RxBuffer[3]=='r')&(RxBuffer[4]=='t')&(RxBuffer[5]=='_')&(RxBuffer[6]=='c')&(RxBuffer[7]=='o')&(RxBuffer[8]=='n')&(RxBuffer[9]=='f'))
		{
			u16 i=0;
			u8 errors=0;
			
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 
	
    // ??????? ????????? ?????????????????? ??????
  //  RCC->BDCR |=  RCC_BDCR_BDRST;
 //   RCC->BDCR &= ~RCC_BDCR_BDRST;
			
			for (i = 0; i < rxsize-10; i += 2)
			{
			  tmp1=RxBuffer[i+10];
				tmp2=RxBuffer[i+11];
				tmp3=0;
				if (tmp1>'9')	
						tmp3=(tmp1-0x37)<<4;
				else
						tmp3=(tmp1-0x30)<<4;
	
				if (tmp2>'9')	
						tmp3+=(tmp2-0x37);
				else
						tmp3+=(tmp2-0x30);				
				*(__IO uint8_t *) (BKPSRAM_BASE + (i>>1)) = tmp3;//*(__IO uint8_t *) ((__IO uint8_t *) (&conf) + i);
				if (*(__IO uint8_t *) (BKPSRAM_BASE + (i>>1)) != tmp3)
						errors=1;
			}

			if (errors==0)
			{
			TxBuffer[0]='w';
			TxBuffer[1]='r';	
			TxBuffer[2]='t';
			TxBuffer[3]='_';					
			TxBuffer[4]='c';		
			TxBuffer[5]='o';	
			TxBuffer[6]='n';	
			TxBuffer[7]='f';	
			TxBuffer[8]='_';	
			TxBuffer[9]='o';
			TxBuffer[10]='k';	
			}
			else
			{
				TxBuffer[0]='w';
				TxBuffer[1]='r';	
				TxBuffer[2]='t';
				TxBuffer[3]='_';					
				TxBuffer[4]='c';		
				TxBuffer[5]='o';	
				TxBuffer[6]='n';	
				TxBuffer[7]='f';	
				TxBuffer[8]='_';	
				TxBuffer[9]='e';
				TxBuffer[10]='r';	
			}
	
			txsize=11;
			tekper=0;
			USART_SendData(USART2, 0x3A);
		}

		if ((RxBuffer[2]=='w')&(RxBuffer[3]=='h')&(RxBuffer[4]=='o')&(RxBuffer[5]=='?'))
		{
			
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 
			
			TxBuffer[0]='u';
			TxBuffer[1]='s';	
			TxBuffer[2]='o';
			TxBuffer[3]='_';					
			TxBuffer[4]='a';		
			TxBuffer[5]='r';	
			TxBuffer[6]='m';	
			TxBuffer[7]='_';	
			TxBuffer[8]='k';	
			TxBuffer[9]='e';
			TxBuffer[10]='y';	
	
			txsize=11;
			tekper=0;
			USART_SendData(USART2, 0x3A);
		}
	
		if ((RxBuffer[2]=='r')&(RxBuffer[3]=='e')&(RxBuffer[4]=='s')&(RxBuffer[5]=='t')&(RxBuffer[6]=='a')&(RxBuffer[7]=='r')&(RxBuffer[8]=='t'))
		{
			
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 
			
			TxBuffer[0]='r';
			TxBuffer[1]='e';	
			TxBuffer[2]='s';
			TxBuffer[3]='t';					
			TxBuffer[4]='a';		
			TxBuffer[5]='r';	
			TxBuffer[6]='t';	
			TxBuffer[7]='_';	
			TxBuffer[8]='o';	
			TxBuffer[9]='k';
	
			txsize=10;
			tekper=0;
			USART_SendData(USART2, 0x3A);
			
			// need restart
		}
	
		if ((RxBuffer[2]=='s')&(RxBuffer[3]=='e')&(RxBuffer[4]=='t')&(RxBuffer[5]=='_')&(RxBuffer[6]=='t')&(RxBuffer[7]=='i')&(RxBuffer[8]=='m')&(RxBuffer[9]=='e'))
		{		
			extern void rtc_SetDate(uint8_t Day, uint8_t Month, uint8_t Year, uint8_t DayOfWeek);
			extern  void rtc_SetTime(uint8_t Hours, uint8_t Minutes, uint8_t Seconds);
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 
			
			TxBuffer[0]='s';
			TxBuffer[1]='e';	
			TxBuffer[2]='t';
			TxBuffer[3]='_';					
			TxBuffer[4]='t';		
			TxBuffer[5]='i';	
			TxBuffer[6]='m';	
			TxBuffer[7]='e';	
			TxBuffer[8]='_';	
			TxBuffer[9]='o';
			TxBuffer[10]='k';
			
			// set time
		// set date
     rtc_SetDate((RxBuffer[14]-0x30)*10+(RxBuffer[15]-0x30), (RxBuffer[12]-0x30)*10+(RxBuffer[13]-0x30), (RxBuffer[10]-0x30)*10+(RxBuffer[11]-0x30), 1);
        
    // set time
    rtc_SetTime((RxBuffer[16]-0x30)*10+(RxBuffer[17]-0x30), (RxBuffer[18]-0x30)*10+(RxBuffer[19]-0x30), (RxBuffer[20]-0x30)*10+(RxBuffer[21]-0x30));
		
			
			txsize=11;
			tekper=0;
			USART_SendData(USART2, 0x3A);
			
			// need restart
		}
		
		if ((RxBuffer[2]=='r')&(RxBuffer[3]=='e')&(RxBuffer[4]=='a')&(RxBuffer[5]=='d'))
		{
			
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 
			
			// test zapis for controlling time on fleshka
			tmp=por;
			TxBuffer[0]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
			tmp%=1000;
			TxBuffer[1]=(uint8_t)(tmp/100)+(uint8_t)0x30;
			tmp%=100;		
			TxBuffer[2]=(uint8_t)(tmp/10)+(uint8_t)0x30;
			tmp%=10;	
			TxBuffer[3]=(uint8_t)(tmp)+(uint8_t)0x30;		
				
			TxBuffer[4]=0x20;	

			// date
			TxBuffer[5]=(uint8_t)(DT1.Year/10)+(uint8_t)0x30;	
			TxBuffer[6]=(uint8_t)(DT1.Year%10)+(uint8_t)0x30;	
			TxBuffer[7]=(uint8_t)(DT1.Month/10)+(uint8_t)0x30;	
			TxBuffer[8]=(uint8_t)(DT1.Month%10)+(uint8_t)0x30;	
			TxBuffer[9]=(uint8_t)(DT1.Day/10)+(uint8_t)0x30;	
			TxBuffer[10]=(uint8_t)(DT1.Day%10)+(uint8_t)0x30;	
			TxBuffer[11]=0x20;				
			
			// time
		
			TxBuffer[12]=(uint8_t)(DT1.Minutes/10)+(uint8_t)0x30;	
			TxBuffer[13]=(uint8_t)(DT1.Minutes%10)+(uint8_t)0x30;	
			TxBuffer[14]=(uint8_t)(DT1.Seconds/10)+(uint8_t)0x30;	
			TxBuffer[15]=(uint8_t)(DT1.Seconds%10)+(uint8_t)0x30;	
			TxBuffer[16]=0x20;
			
			TxBuffer[17]=sost_pribl+0x30;
			TxBuffer[18]=0x20;
			// zn from adc with calibr and averaging

			tmp=ADC3ConvertedValue;
			TxBuffer[19]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
			tmp%=1000;
			TxBuffer[20]=(uint8_t)(tmp/100)+(uint8_t)0x30;
			tmp%=100;		
			TxBuffer[21]=(uint8_t)(tmp/10)+(uint8_t)0x30;
			tmp%=10;	
			TxBuffer[22]=(uint8_t)(tmp)+(uint8_t)0x30;
			TxBuffer[23]=0x20;		
			
			tmp=fz_average[0];
			TxBuffer[24]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
			tmp%=1000;
			TxBuffer[25]=(uint8_t)(tmp/100)+(uint8_t)0x30;
			tmp%=100;		
			TxBuffer[26]=(uint8_t)(tmp/10)+(uint8_t)0x30;
			tmp%=10;	
			TxBuffer[27]=(uint8_t)(tmp)+(uint8_t)0x30;		

			txsize=28;
			tekper=0;
			USART_SendData(USART2, 0x3A);
		}		
		
		if ((RxBuffer[2]=='r')&(RxBuffer[3]=='d')&(RxBuffer[4]=='_')&(RxBuffer[5]=='c')&(RxBuffer[6]=='o')&(RxBuffer[7]=='n')&(RxBuffer[8]=='f'))
		{
			u16 i=0;
			extern st_conf conf;		
			
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 

			
			
			/*
			txsize=sizeof(st_conf)<<1;
	
			for (i = 0; i < (txsize); i += 1)
			{
				u16 tmp=0;
	//			TxBuffer[i]=(*(__IO uint8_t *) ((__IO uint8_t *) (&conf) + i));
				tmp=(*(__IO uint8_t *) ((__IO uint8_t *) (&conf) + i));
				TxBuffer[i<<1]=(tmp>>8)+0x30;
				TxBuffer[(i<<1)+1]=tmp+0x30;
			}	
*/

			txsize=sizeof(st_conf);
	
			for (i = 0; i < (txsize); i += 1)
			{
				TxBuffer[i]=(*(__IO uint8_t *) ((__IO uint8_t *) (&conf) + i));
			}	
			tekper=0;
			USART_SendData(USART2, 0x3A);
		}		
		
		new_komand=0;
	}
	

	
				//	indicators[0].chislo=1234;
		//		indicators[1].chislo=1234;
		//		indicate (0);
		//		indicate (1);
				
}

	


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles External line 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  /* Check the clic on the accelerometer to Pause/Resume Playing */
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    if( Count==1)
    {
      PauseResumeStatus = 1;
      Count = 0;
    }
    else
    {
      PauseResumeStatus = 0;
      Count = 1;
    }
    /* Clear the EXTI line 1 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}

/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
   uint8_t clickreg = 0;

	
  /* Checks whether the TIM interrupt has occurred */
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		/*
    if( LED_Toggle == 3)
    {
      // LED3 Orange toggling 
      STM_EVAL_LEDToggle(LED3);
      STM_EVAL_LEDOff(LED6);
      STM_EVAL_LEDOff(LED4);
    }
    else if( LED_Toggle == 4)
    {
      // LED4 Green toggling 
      STM_EVAL_LEDToggle(LED4);
      STM_EVAL_LEDOff(LED6);
      STM_EVAL_LEDOff(LED3);
    }
    else if( LED_Toggle == 6)
    {
      // LED6 Blue toggling 
      STM_EVAL_LEDOff(LED3);
      STM_EVAL_LEDOff(LED4);
      STM_EVAL_LEDToggle(LED6);
    }
    else if (LED_Toggle ==0)
    {
      // LED6 Blue On to signal Pause 
      STM_EVAL_LEDOn(LED6);
    }
    else if (LED_Toggle == 7)
    {
      // LED4 toggling with frequency = 439.4 Hz 
      STM_EVAL_LEDOff(LED3);
      STM_EVAL_LEDOff(LED4);
      STM_EVAL_LEDOff(LED5);
      STM_EVAL_LEDOff(LED6);
    }
		*/
	

    capture = TIM_GetCapture1(TIM4);
    TIM_SetCompare1(TIM4, capture + CCR_Val);
  }
}

#if defined MEDIA_USB_KEY
/**
  * @brief  EXTI0_IRQHandler
  *         This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  /* Checks whether the User Button EXTI line is asserted*/
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
  { 
    if (Command_index == 1)
    {
      RepeatState = 0;
      /* Switch to play command */
      Command_index = 0;
    }
    else if (Command_index == 0)
    {
      /* Switch to record command */
      Command_index = 1;
  //    XferCplt = 1;
  //    EVAL_AUDIO_Stop(CODEC_PDWN_SW);
    }
    else
    {
      RepeatState = 0;
      /* Switch to play command */
      Command_index = 0; 
    }
  } 
  /* Clears the EXTI's line pending bit.*/ 
  EXTI_ClearITPendingBit(EXTI_Line0);
}


/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  USB_OTG_BSP_TimerIRQ();
}


/**
  * @brief  This function handles USB-On-The-Go FS global interrupt request.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
  USBH_OTG_ISR_Handler(&USB_OTG_Core);
}
#endif /* MEDIA_USB_KEY */

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @brief  This function handles RCC interrupt request. 
  * @param  None
  * @retval None
  */
void RCC_IRQHandler(void)
{
  if(RCC_GetITStatus(RCC_IT_HSERDY) != RESET)
  { 
    /* Clear HSERDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_HSERDY);

    /* Check if the HSE clock is still available */
    if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
    { 
      /* Enable PLL: once the PLL is ready the PLLRDY interrupt is generated */ 
      RCC_PLLCmd(ENABLE);     
    }
  }

  if(RCC_GetITStatus(RCC_IT_PLLRDY) != RESET)
  { 
    /* Clear PLLRDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_PLLRDY);

    /* Check if the PLL is still locked */
    if (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != RESET)
    { 
      /* Select PLL as system clock source */
      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    }
  }
}

/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
