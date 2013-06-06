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
		u16 tmp=0;
		del=0;
		tick++;
		time_label=tick;

		
		rtc_Get(&DT1);
		
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
		tmp=por;
		TxBuffer[0]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
		tmp%=1000;
		TxBuffer[1]=(uint8_t)(tmp/100)+(uint8_t)0x30;
		tmp%=100;		
		TxBuffer[2]=(uint8_t)(tmp/10)+(uint8_t)0x30;
		tmp%=10;	
		TxBuffer[3]=(uint8_t)(tmp)+(uint8_t)0x30;		
			
		TxBuffer[4]=0x20;	
		
	
		TxBuffer[5]=(uint8_t)(DT1.Minutes/10)+(uint8_t)0x30;	
		TxBuffer[6]=(uint8_t)(DT1.Minutes%10)+(uint8_t)0x30;	
		TxBuffer[7]=(uint8_t)(DT1.Seconds/10)+(uint8_t)0x30;	
		TxBuffer[8]=(uint8_t)(DT1.Seconds%10)+(uint8_t)0x30;	
		
		
		tmp=fz_average[0];
		TxBuffer[9]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
		tmp%=1000;
		TxBuffer[10]=(uint8_t)(tmp/100)+(uint8_t)0x30;
		tmp%=100;		
		TxBuffer[11]=(uint8_t)(tmp/10)+(uint8_t)0x30;
		tmp%=10;	
		TxBuffer[12]=(uint8_t)(tmp)+(uint8_t)0x30;		
	//	GPIO_WriteBit(GPIOD, tx_pin_en, Bit_SET);      //   GPIOB.2
	//	GPIO_WriteBit(GPIOD, rx_pin_en, Bit_RESET);    //   GPIOB.2
		txsize=13;
		tekper=0;
		GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 
		USART_SendData(USART2, 0x3A);
		
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
	 
 indicate_lin(0,(u16)tick, 1680, 28);
	  // indicate(0,0);
 indicate(1,(u16)tick);		
		 
	 }		 
		
	if ((tick%60)==0)
	{
		minute++;
	}
	}
	
	
	if (new_komand)
	{
	//	RxBuffer[tekpr];
		
		
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
