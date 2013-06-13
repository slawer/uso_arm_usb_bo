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
	
	
	
	// ��� � 10 ��
	// ������� ������� ��������
		summa[0]+=ADC3ConvertedValue;

	 
//		summa[0]+=100;
		kol_average++;
	
	if (kol_average==10)
	{
	
		average[0]=summa[0]/kol_average;
		kol_average=0;
		summa[0]=0;
		
		
		// ��� � 100 ��
		// ��������� ���������� �������� 
	
		fz[0]=fiz_vel(average[0],0);
	
		// ��������� ���� �� ������������
		test_rele(fz[0], 0);	
	
	// ������� ������� �������� �� ���������� �������
		fz_average[0]=moving_average(fz[0],0);
		
		if (fz_average[0]>max[0])
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
		extern st_conf conf;
		
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
	 
// indicate_lin(0,(u16)fz_average[0], 4096);
// indicate(1,(u16)(fz_average[0]/10));
	 
	 indicate(1,(u16)(max[0]/10),3);   																// maximum
	 indicate_lin(2,(u16) fz_average[0], (u16) conf.lin.max1, (u16) conf.lin.kol_st);			// lineika 
	 indicate(3,(u16)(fz_average[0]/10),3);														// tek
	 indicate_time(4,(u8)DT1.Hours,(u8) DT1.Minutes);								//	time
		 
	 }		 
		
	if ((tick%60)==0)
	{
		minute++;
	}
	}
	
	
	if (new_komand)
	{
		u16 tmp=0;
	
		// wrt_conf
		if ((RxBuffer[2]=='w')&(RxBuffer[3]=='r')&(RxBuffer[4]=='t')&(RxBuffer[5]=='_')&(RxBuffer[6]=='c')&(RxBuffer[7]=='o')&(RxBuffer[8]=='n')&(RxBuffer[9]=='f'))
		{
			u16 i=0;
			u8 errors=0;
			
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 
	
		
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

		// who?
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
	
		// restart
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
			
			SCB->AIRCR = 0x05FA0004;
			
			
			
			// need restart
		}
	
		// set_time
		if ((RxBuffer[2]=='s')&(RxBuffer[3]=='e')&(RxBuffer[4]=='t')&(RxBuffer[5]=='_')&(RxBuffer[6]=='t')&(RxBuffer[7]=='i')&(RxBuffer[8]=='m')&(RxBuffer[9]=='e'))
		{		
			extern void rtc_SetDate(uint8_t Day, uint8_t Month, uint8_t Year, uint8_t DayOfWeek);
			extern  void rtc_SetTime(uint8_t Hours, uint8_t Minutes, uint8_t Seconds);
			extern  void rtc_Unlock(void);
			extern void rtc_Lock(void);
			
			uint32_t Tens, Units;
      uint32_t TempReg = 0;
			u8 i=0;
			
			
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 


			// set time
		rtc_Unlock();
		RTC->ISR |= RTC_ISR_INIT;
		while(!(RTC->ISR & RTC_ISR_INITF)) {}

		RTC->PRER = 263; //Sync;         
    RTC->PRER =263 | (127<<16); //Sync | (Async << 16);
			
			
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
rtc_SetTime((RxBuffer[16]-0x30)*10+(RxBuffer[17]-0x30), (RxBuffer[18]-0x30)*10+(RxBuffer[19]-0x30), (RxBuffer[20]-0x30)*10+(RxBuffer[21]-0x30));

rtc_SetDate((RxBuffer[10]-0x30)*10+(RxBuffer[11]-0x30), (RxBuffer[12]-0x30)*10+(RxBuffer[13]-0x30), (RxBuffer[14]-0x30)*10+(RxBuffer[15]-0x30),1);
/*
if (1)
{

    TempReg = 0;
    {
        Tens  = ((RxBuffer[10]-0x30) / 10) & 0x0f;          // ??????? ???
        Units = (RxBuffer[11]-0x30) & 0x0f; // ??????? ???
        
        TempReg |= (Tens  << 20); // YT, 20
        TempReg |= (Units << 16); // YU, 16
    }
    {
        Tens  = (RxBuffer[12]-0x30) & 0x01;          // ??????? ???????
        Units = (RxBuffer[13]-0x30) & 0x0f; // ??????? ???????
        
        TempReg |= (Tens  << 12); // MT, 12
        TempReg |= (Units << 8);  // MU, 8
    }
    {
        Tens  = (RxBuffer[14]-0x30) & 0x03;          // ??????? ????
        Units = (RxBuffer[15]-0x30) & 0x0f; // ??????? ????
        
        TempReg |= (Tens  << 4); // DT, 4
        TempReg |= (Units << 0);  // DU, 0
    }
    {
        TempReg |= ((1 & 0x07) << 13); // WDU, 13
    }
    RTC->DR = TempReg;	
	}	
*/
		RTC->CR |= RTC_CR_FMT;
    RTC->ISR &= ~RTC_ISR_INIT;
		rtc_Lock();

	
		rtc_Get(&DT1);
//			while (TempReg!=RTC->DR)
		while (((RxBuffer[10]-0x30)*10+(RxBuffer[11]-0x30))!=DT1.Day)
				rtc_Get(&DT1);

				
			txsize=11;
			tekper=0;
			USART_SendData(USART2, 0x3A);
			
			// need restart
		}
		
		// read
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
			TxBuffer[12]=(uint8_t)(DT1.Hours/10)+(uint8_t)0x30;	
			TxBuffer[13]=(uint8_t)(DT1.Hours%10)+(uint8_t)0x30;			
			TxBuffer[14]=(uint8_t)(DT1.Minutes/10)+(uint8_t)0x30;	
			TxBuffer[15]=(uint8_t)(DT1.Minutes%10)+(uint8_t)0x30;	
			TxBuffer[16]=(uint8_t)(DT1.Seconds/10)+(uint8_t)0x30;	
			TxBuffer[17]=(uint8_t)(DT1.Seconds%10)+(uint8_t)0x30;	
			TxBuffer[18]=0x20;
			
			TxBuffer[19]=sost_pribl+0x30;
			TxBuffer[20]=0x20;
			// zn from adc with calibr and averaging

			tmp=ADC3ConvertedValue;
			TxBuffer[21]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
			tmp%=1000;
			TxBuffer[22]=(uint8_t)(tmp/100)+(uint8_t)0x30;
			tmp%=100;		
			TxBuffer[23]=(uint8_t)(tmp/10)+(uint8_t)0x30;
			tmp%=10;	
			TxBuffer[24]=(uint8_t)(tmp)+(uint8_t)0x30;
			TxBuffer[25]=0x20;		
			
			tmp=fz_average[0];
			TxBuffer[26]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
			tmp%=1000;
			TxBuffer[27]=(uint8_t)(tmp/100)+(uint8_t)0x30;
			tmp%=100;		
			TxBuffer[28]=(uint8_t)(tmp/10)+(uint8_t)0x30;
			tmp%=10;	
			TxBuffer[29]=(uint8_t)(tmp)+(uint8_t)0x30;		
			TxBuffer[30]=0x20;
			
			tmp=max[0];
			TxBuffer[31]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
			tmp%=1000;
			TxBuffer[32]=(uint8_t)(tmp/100)+(uint8_t)0x30;
			tmp%=100;		
			TxBuffer[33]=(uint8_t)(tmp/10)+(uint8_t)0x30;
			tmp%=10;	
			TxBuffer[34]=(uint8_t)(tmp)+(uint8_t)0x30;		
			
			txsize=35;
			tekper=0;
			USART_SendData(USART2, 0x3A);
		}		
		
		// read_conf
		if ((RxBuffer[2]=='r')&(RxBuffer[3]=='d')&(RxBuffer[4]=='_')&(RxBuffer[5]=='c')&(RxBuffer[6]=='o')&(RxBuffer[7]=='n')&(RxBuffer[8]=='f'))
		{
			u16 i=0;
			extern st_conf conf;		
			
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 
/*
		u8 address;
		u8 ver_po_st;
		u8 ver_po_ml;
	  u8 tek_gr_kal; 
	  u8 tm_antidreb;
		u8 revers_group_select;
		u8 revers_peredacha_select;
		u8 rez8;
	
		u16	per_usr;
		u16	time_max;
		u16 por_rele;
		u16 tm_rele_on;
		u16 tm_rele_off;
		u16 rez16;
	*/	
/*
			conf.address=1;
			conf.ver_po_st=2;
			conf.ver_po_ml=3;
			conf.tek_gr_kal=4;
			conf.tm_antidreb=5;
			conf.revers_group_select=6;
			conf.revers_peredacha_select=7;
			conf.rez8=8;
			conf.per_usr=9;
			conf.time_max=10;
			conf.por_rele=11;
			conf.tm_rele_on=12;
			conf.tm_rele_off=13;
			conf.rez16=14;
			
		conf.lin.kol_st=15;
		conf.lin.max1=16;	
		conf.lin.max2=17;
		conf.lin.max3=18;
		conf.lin.max4=19;
		*/
			/*
			conf.indicators[0].numb=15;
			conf.indicators[0].kol_cifr=16;
			conf.indicators[0].type_ind=17;
			conf.indicators[0].yark=18;
			conf.indicators[0].rez_viv=19;
			conf.indicators[0].pol_zap=20;
			conf.indicators[0].r1=21;
			conf.indicators[0].r2=22;
			conf.indicators[0].chislo=23;
			conf.indicators[0].porog=24;

			conf.indicators[1].numb=25;
			conf.indicators[1].kol_cifr=26;
			conf.indicators[1].type_ind=27;
			conf.indicators[1].yark=28;
			conf.indicators[1].rez_viv=29;
			conf.indicators[1].pol_zap=30;
			conf.indicators[1].r1=31;
			conf.indicators[1].r2=32;
			conf.indicators[1].chislo=33;
			conf.indicators[1].porog=34;
		

		

		
			conf.indicators[2].numb=35;
			conf.indicators[2].kol_cifr=36;
			conf.indicators[2].type_ind=37;
			conf.indicators[2].yark=38;
			conf.indicators[2].rez_viv=39;
			conf.indicators[2].pol_zap=40;
			conf.indicators[2].r1=41;
			conf.indicators[2].r2=42;
			conf.indicators[2].chislo=43;
			conf.indicators[2].porog=44;
			
			conf.indicators[3].numb=45;
			conf.indicators[3].kol_cifr=46;
			conf.indicators[3].type_ind=47;
			conf.indicators[3].yark=48;
			conf.indicators[3].rez_viv=49;
			conf.indicators[3].pol_zap=50;
			conf.indicators[3].r1=51;
			conf.indicators[3].r2=52;
			conf.indicators[3].chislo=53;
			conf.indicators[3].porog=54;
				*/
		/*		
			conf.gr_kal1.tabl1.fz[0]=55;
			conf.gr_kal1.tabl1.fz[1]=56;
			conf.gr_kal1.tabl1.fz[2]=57;
			conf.gr_kal1.tabl1.fz[3]=58;
			conf.gr_kal1.tabl1.fz[4]=59;
			conf.gr_kal1.tabl1.fz[5]=60;
			conf.gr_kal1.tabl1.fz[6]=61;
			conf.gr_kal1.tabl1.fz[7]=62;
			conf.gr_kal1.tabl1.fz[8]=63;
			conf.gr_kal1.tabl1.fz[9]=64;

			conf.gr_kal1.tabl1.kod[0]=65;
			conf.gr_kal1.tabl1.kod[1]=66;
			conf.gr_kal1.tabl1.kod[2]=67;
			conf.gr_kal1.tabl1.kod[3]=68;
			conf.gr_kal1.tabl1.kod[4]=69;
			conf.gr_kal1.tabl1.kod[5]=70;
			conf.gr_kal1.tabl1.kod[6]=71;
			conf.gr_kal1.tabl1.kod[7]=72;
			conf.gr_kal1.tabl1.kod[8]=73;
			conf.gr_kal1.tabl1.kod[9]=74;
	
			conf.gr_kal1.tabl2.fz[0]=75;
			conf.gr_kal1.tabl2.fz[1]=76;
			conf.gr_kal1.tabl2.fz[2]=77;
			conf.gr_kal1.tabl2.fz[3]=78;
			conf.gr_kal1.tabl2.fz[4]=79;
			conf.gr_kal1.tabl2.fz[5]=80;
			conf.gr_kal1.tabl2.fz[6]=81;
			conf.gr_kal1.tabl2.fz[7]=82;
			conf.gr_kal1.tabl2.fz[8]=83;
			conf.gr_kal1.tabl2.fz[9]=84;

			conf.gr_kal1.tabl2.kod[0]=85;
			conf.gr_kal1.tabl2.kod[1]=86;
			conf.gr_kal1.tabl2.kod[2]=87;
			conf.gr_kal1.tabl2.kod[3]=88;
			conf.gr_kal1.tabl2.kod[4]=89;
			conf.gr_kal1.tabl2.kod[5]=90;
			conf.gr_kal1.tabl2.kod[6]=91;
			conf.gr_kal1.tabl2.kod[7]=92;
			conf.gr_kal1.tabl2.kod[8]=93;
			conf.gr_kal1.tabl2.kod[9]=94;	
			
			// dr2
			conf.gr_kal2.tabl1.fz[0]=95;
			conf.gr_kal2.tabl1.fz[1]=96;
			conf.gr_kal2.tabl1.fz[2]=97;
			conf.gr_kal2.tabl1.fz[3]=98;
			conf.gr_kal2.tabl1.fz[4]=99;
			conf.gr_kal2.tabl1.fz[5]=100;
			conf.gr_kal2.tabl1.fz[6]=101;
			conf.gr_kal2.tabl1.fz[7]=102;
			conf.gr_kal2.tabl1.fz[8]=103;
			conf.gr_kal2.tabl1.fz[9]=104;

			conf.gr_kal2.tabl1.kod[0]=105;
			conf.gr_kal2.tabl1.kod[1]=106;
			conf.gr_kal2.tabl1.kod[2]=107;
			conf.gr_kal2.tabl1.kod[3]=108;
			conf.gr_kal2.tabl1.kod[4]=109;
			conf.gr_kal2.tabl1.kod[5]=100;
			conf.gr_kal2.tabl1.kod[6]=111;
			conf.gr_kal2.tabl1.kod[7]=112;
			conf.gr_kal2.tabl1.kod[8]=113;
			conf.gr_kal2.tabl1.kod[9]=114;
	
			conf.gr_kal2.tabl2.fz[0]=115;
			conf.gr_kal2.tabl2.fz[1]=116;
			conf.gr_kal2.tabl2.fz[2]=117;
			conf.gr_kal2.tabl2.fz[3]=118;
			conf.gr_kal2.tabl2.fz[4]=119;
			conf.gr_kal2.tabl2.fz[5]=120;
			conf.gr_kal2.tabl2.fz[6]=121;
			conf.gr_kal2.tabl2.fz[7]=122;
			conf.gr_kal2.tabl2.fz[8]=123;
			conf.gr_kal2.tabl2.fz[9]=124;

			conf.gr_kal2.tabl2.kod[0]=125;
			conf.gr_kal2.tabl2.kod[1]=126;
			conf.gr_kal2.tabl2.kod[2]=127;
			conf.gr_kal2.tabl2.kod[3]=128;
			conf.gr_kal2.tabl2.kod[4]=129;
			conf.gr_kal2.tabl2.kod[5]=130;
			conf.gr_kal2.tabl2.kod[6]=131;
			conf.gr_kal2.tabl2.kod[7]=132;
			conf.gr_kal2.tabl2.kod[8]=133;
			conf.gr_kal2.tabl2.kod[9]=134;	
*/		
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

			txsize=sizeof(st_conf);  // dl = 228
	
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
