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

#include "stm32f4xx_flash.h"

extern void FLASH_Unlock(void);
extern void FLASH_Lock(void);
extern FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange);
extern FLASH_Status FLASH_EraseAllSectors(uint8_t VoltageRange);
extern FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint64_t Data);
extern FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
extern FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
extern FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data);


extern void start_ds();
extern void write_bait_ds();
extern void stop_ds();
extern void sleep();
extern void wr_ack_ds();
extern void read_ds(u8 kol);
extern u8 read_bait_ds();


u32 tmp_sec=0;
u32 sec=0,  days=0;
u8 kon_sut=0;
u16 kol_time_korr=0;

u32 pred_tick=0, tek_max_min=0;

u16 kol_cifr=0;

u16	kol_rele_on=0;
u16	kol_rele_off=0;
u16 time_max=0;
u8 tk_null=0;

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

/*
//for zap vo flash

#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)
void flash_unlock(void) {
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}


void flash_lock() {
  FLASH->CR |= FLASH_CR_LOCK;
}


//when can write
uint8_t flash_ready(void) {
  return !(FLASH->SR & FLASH_SR_BSY);
}
 
 
//  erase 1 page
void flash_erase_page(uint32_t address) {
  FLASH->CR|= FLASH_CR_PER; //????????????? ??? ???????? ????? ????????
  FLASH->AR = address; // ?????? ?? ?????
  FLASH->CR|= FLASH_CR_STRT; // ????????? ???????? 
  while(!flash_ready())
    ;  //???? ???? ???????? ????????. 
  FLASH->CR&= ~FLASH_CR_PER; //?????????? ??? ???????
}

// write 4 bait
void flash_write(uint32_t address,uint32_t data) {
  FLASH->CR |= FLASH_CR_PG; //????????? ???????????????? ?????
  while(!flash_ready()) //??????? ?????????? ????? ? ??????
    ;
  *(__IO uint16_t*)address = (uint16_t)data; //????? ??????? 2 ????
  while(!flash_ready())
    ;
  address+=2;
  data>>=16;
  *(__IO uint16_t*)address = (uint16_t)data; //????? ??????? 2 ?????
  while(!flash_ready())
    ;
  FLASH->CR &= ~(FLASH_CR_PG); //????????? ???????????????? ?????
}
*/

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

u16 fiz_vel(u16 kod, st_tab_kal* tk)
{
	u8 i=0, nul=0;
	st_tab_kal *tab = tk;
//	tab_kal
	/*
	    
       
                
                if kod[1:]==[0,0,0,0,0,0,0,0,0]:
                    print 'no tabl' 
                    return kodadc
                    
                if kodadc<kod[0]:
                    return fz[0]
                
                for i in range(1,10):
                    
                    if kodadc<kod[i]:
                        if i==0:
                            print 'i=0',fz[0]
                            return fz[0]
                        else:
                            if kod[i]==0:
                                print 'kod=0',fz[i-1]
                                return fz[i-1]
                            else:
                                print 'norm',
                                return (fz[i]-fz[i-1])*(kodadc-kod[i-1])/(kod[i]-kod[i-1])+fz[i-1]
                                
                    if  kod[i-1]>0 and kod[i]==0:
                        return fz[i-1]
                
                return fz[9]

	*/
	
	
	/*	
	conf.gr_kal1.tabl1.fz[0]
	conf.gr_kal1.tabl1.kod[0]=65;
  conf.tek_gr_kal=4;
  sost_pribl	
	tk
	*/
		tk_null=0;
		for (i = 1; i < 10; i += 1)
			if (tk->kod[i]==0) 
					nul++;	
	  if (nul==9)
		{		tk_null=1;
				return kod; }
		
		if (kod<tk->kod[0])
        return tk->fz[0];
		
		for (i = 1; i < 10; i += 1)	
		{
			float tmp=0;
			if (kod<tk->kod[i]) {
					if (i==0)
							return tk->fz[0];
					else  {
							if (tk->kod[i]==0)
									return tk->fz[i-1];
							else  {
								  tmp=(tk->fz[i]-tk->fz[i-1])*(kod-tk->kod[i-1])/(tk->kod[i]-tk->kod[i-1])+tk->fz[i-1]+0.5;
									return (u16) tmp;  } } }
									
			if  ((tk->kod[i-1]>0)&(tk->kod[i]==0))
					return tk->fz[i-1];
  	}	
    
		return tk->fz[9];		
}


u16 moving_average(u16 kod, u8 numb)
{
	extern u16 tek_kol;
	extern u16 kol_usr;
	extern u32 buf_sum;
	u16 tmp=0;
	
	if (kol_usr==0)
		return kod;

	if (kol_usr==1)
		return kod;
	
	if (tek_kol<kol_usr)
	{
		buf_sum+=kod;
		tek_kol++;
		return (u16)((buf_sum/tek_kol)+0.5);
	}
	else
	{
		buf_sum+=kod;
		tmp=(u16) ((buf_sum/tek_kol)+0.5);
		buf_sum-=tmp;
		return  tmp;
	}
}

u8 test_rele(u16 fz, u8 numb)
{
	extern st_conf conf;
	
	if (fz>conf.por_rele)
			kol_rele_on++;			
	else
			kol_rele_off++;
		
	if (kol_rele_on>=conf.tm_rele_on)
	{
		kol_rele_on=0;
		kol_rele_off=0;
		avariya=1;
		PORT_AVARIYA->BSRRL = PIN_AVARIYA;	// on PIN_AVARIYA		
		PORT_RELE->BSRRL = PIN_RELE;	// on PIN_RELE		
	}
	
	if (kol_rele_off>=conf.tm_rele_off)
	{
		kol_rele_on=0;
		kol_rele_off=0;
		avariya=0;
		PORT_AVARIYA->BSRRH = PIN_AVARIYA;  	// off  PIN_AVARIYA
		PORT_RELE->BSRRH = PIN_RELE;  	// off  PIN_RELE		
	}
}


void update_indicators()
{
		extern st_conf conf;
		extern u8 bufout[20];
//		u8 bufer[8]={0x40,0x60,0x70,0x78,0x7C,0x7E,0x7F,0xFF};
	//	u8 bufer[8]={0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF};

		u8 bufer[9]={0,1,3,7,0xf,0x1f,0x3f,0x7f,0xff};
		// start indicators
	
				if (tick!=pred_tick)
				{
					if ((tick%2)==0)
				//	if ((kol_average%2)==0)
					//		indicate_time(4,(u8)DT1.Hours,(u8) DT1.Minutes,1);				//	time	
							indicate_time(4,(u8) bufout[2],(u8) bufout[1],1);				//	time	
					else
					//		indicate_time(4,(u8)DT1.Hours,(u8) DT1.Minutes,0);				//	time	 
							indicate_time(4,(u8) bufout[2],(u8) bufout[1],0);				//	time	 
					pred_tick=tick;
								
			/*		
				//				indicate(1,(u16)(u16)(kol_cifr*1111),4);
								indicate_test	(1,(u16)(bufer[kol_cifr]),4);
							
								if ((tick%5==0))
								{

										kol_cifr++;
									if  (kol_cifr>8)
										kol_cifr=0;
								}
			*/
				}

	
				 if (tk_null==1)
				 {
					 indicate_err(1);   	// tek
					 indicate_err(2);			// lineika 
					 indicate_err(3);			// maximum
				 }
				 else
				 {		
					 if ((avariya==1)&(del%5>3))
					 {
							ind_blank_all(1); 
							ind_blank_all(2); 
							ind_blank_all(3); 	 
						}
						else
						{
								
						 // dop usrednenie na vivod indicatorov???
							indicate(2,(u16)(fz_average[0]),3);   							// tek
							
							indicate(3,(u16)(max[0]),3);														// maximum
							
							if (conf.tek_gr_kal==conf.revers_group_select)
									if (sost_pribl==0)  {
										indicate_lin(1,(u16) fz_average[0], (u16) conf.lin.max1, (u16) conf.lin.kol_st);		}	// lineika 
									else {
										indicate_lin(1,(u16) fz_average[0], (u16) conf.lin.max2, (u16) conf.lin.kol_st);		}	// lineika 
							else
									if (sost_pribl==0) {
										indicate_lin(1,(u16) fz_average[0], (u16) conf.lin.max3, (u16) conf.lin.kol_st);		}	// lineika 
									else {
										indicate_lin(1,(u16) fz_average[0], (u16) conf.lin.max4, (u16) conf.lin.kol_st);		}	// lineika 
					
						//	indicate_lin(1,(u16) fz_average[0], (u16) 1000, (u16) conf.lin.kol_st);
						//		indicate_lin(1,(u16) kol_cifr*1111, (u16) 1000, (u16) conf.lin.kol_st);
								

							
					 }	
				}

			// end indicators
						
}

void TIM6_DAC_IRQHandler(){

    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {

					
	    update_indicators();

			
      TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
			TIM_Cmd(TIM6, DISABLE);  // stop timer
    }
}


u32 date_to_sec(u8 yy, u8 mm, u8 dd, u8 hh, u8 mn, u8 ss)
{
		extern u8 bufout[20];
		
		u8 i, mas[12]={31,28,31,30,31,30,31,31,30,31,30,31};
	
//	sec=ss+mn*60+hh*3600+dd*86400;
	
//	31536000 - в обычном
//	31104000 - в високосном
	

 /*
		days=yy * 365;
		if (yy%4==0)
			days+=yy/4;
		else
			days+=yy/4+1;
		
		for (i=1; i<mn; i++)
				days+=mas[i-1];
		if ((mn>2) & (yy%4==0))
				days+=1;
		
		days+=dd;
		sec=days*86400+ss+mm*60+hh*3600;
*/	
		days=bufout[6] * 365;
		if (bufout[6]%4==0)
			days+=bufout[6]/4;
		else
			days+=bufout[6]/4+1;
		
		for (i=1; i<bufout[5]; i++)
				days+=mas[i-1];
		if ((bufout[5]>2) & (bufout[6]%4==0))
				days+=1;
		
		days+=bufout[4];
		sec=days*86400+bufout[0]+bufout[1]*60+bufout[2]*3600;		
/*
	 den = yy * 365 + god_l2;
	 switch(mm)
		{
			case 1 : den +=31; break;
			case 2 : den +=69; break;
			case 3 : den +=90; break;
			case 4 : den +=120; break;
			case 5 : den +=151; break;
			case 6 : den +=181; break;
			case 7 : den +=212; break;
			case 8 : den +=243; break;
			case 9 : den +=273; break;
			case 10 : den +=304; break;
			case 11 : den +=334; break;
			case 12 : den +=365; break;
		 };
	 if ((yy % 4)&(mm>2)) den++;
	 den+=dd;
	 sec = den * 86400;
	 sec += hh * 3600;
	 sec += mn * 60;
	 sec += ss;
	 */
	 
	 return sec;
}


void sec_to_date(u32 sec)
{
	u8  i=0, km=0;
	int k=0;
	extern u8  bufout[20];
	u8 mas[12]={31,28,31,30,31,30,31,31,30,31,30,31};
	
 	bufout[0]=sec%60;	// sec
	sec=sec/60;
	bufout[1]=sec%60;   // min
	sec=sec/60;
	bufout[2]=sec%24;    // hour
	sec=sec/24;

	i=0;
	while (1)
	{
			if (i%4==0)
					k=366;
			else
					k=365;
			if (sec>k)
			{
					sec-=k;
					i+=1;
			}
			else
					break;
		}
     
		bufout[6]=i;		// year

		km=1;
		while (1)
		{
				i=mas[km-1];
				if ((km==2)&(bufout[6]%4==0))
						i+=1;
				if (sec>i)
				{
						sec-=i;
						km+=1;
				}
				else
						break;
		}
		bufout[5]=km;
		bufout[4]=sec;
                        
                      
/*												
	bufout[0]=sec%60;	// sec
	sec=sec/60;
	bufout[1]=sec%60;   // min
	sec=sec/60;
	bufout[2]=sec%24;    // hour
	sec=sec/24;

	
	i=0;
	while (1)  
	{
		if 	(i%4==0)
			k=366;				
		else 
			k=365;
		
		if (sec>k)
		{
			sec-=k;
			i+=1;
		}
		else
			break;
	}
	bufout[6]=i;
	
	i=0;
	if (bufout[6]%4==0)
			mas[1]=29;
	while (sec>mas[i])  {
		sec-=mas[i];
		i++;							}
			 
	bufout[4]=sec;   // day
	bufout[5]=i;   // month
		*/
	
}

/**
  * @brief  This function handles SysTick Handler.  10ms
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	extern __IO uint16_t ADC3ConvertedValue;
	extern u32 tick;
	extern st_conf conf;

	extern u8 b_err_cl, bufout[20], zbuf[20], error_ds, fl_need_correct_ds;
	
	
	PORT_Conrtol->BSRRL = PIN_Conrtol;
	
 //  проверка состояния датчика приближений - 
	if (conf.revers_peredacha_select==0)
		if ((PORT_PRIBL->IDR & PIN_PRIBL)==0)
		{
			kol_pribl_vikl++;
		}
		else
		{
			kol_pribl_vkl++;
		}
	else
	{
		if ((PORT_PRIBL->IDR & PIN_PRIBL)!=0)
			{
				kol_pribl_vikl++;
			}
			else
			{
				kol_pribl_vkl++;
			}
	}
	
//	conf.tm_antidreb=2;
	if (kol_pribl_vkl>=conf.tm_antidreb)
	{
			sost_pribl=1;	
			PORT_PER_NIZ->BSRRH = PIN_PER_NIZ;  	// off  PIN_PER_NIZ
			PORT_PER_VERH->BSRRL = PIN_PER_VERH;	// on PIN_PER_VERH
			kol_pribl_vkl=0;
		  kol_pribl_vikl=0;
	} 
	
	if (kol_pribl_vikl>=conf.tm_antidreb)
	{
			sost_pribl=0;	
			PORT_PER_NIZ->BSRRL = PIN_PER_NIZ;  	// on  PIN_PER_NIZ
			PORT_PER_VERH->BSRRH = PIN_PER_VERH;	// off PIN_PER_VERH
			kol_pribl_vkl=0;
			kol_pribl_vikl=0;
	}

	
	// проверка кнопок-состояния переключателей группы калибровок
	// on 	GPIO_PORT[Led]->BSRRL = GPIO_PIN[Led];
	// off  GPIO_PORT[Led]->BSRRH = GPIO_PIN[Led];    
//	conf.tek_gr_kal

		if ((PORT_K1->IDR & PIN_K1)==0)
		{
			kol_gr1_vkl++;
		}

		if ((PORT_K2->IDR & PIN_K2)==0)
		{
			kol_gr2_vkl++;
		}

	//	conf.tm_antidreb=2;
		if (kol_gr1_vkl>=conf.tm_antidreb)
		{
				conf.tek_gr_kal=0;
				new_group=1;
		 // gr 1
				PORT_L1->BSRRL = PIN_L1;  	// on  PIN_L1
				PORT_L2->BSRRH = PIN_L2;	  // off PIN_L2

				kol_gr1_vkl=0;
				kol_gr2_vkl=0;
		}		

		if (kol_gr2_vkl>=conf.tm_antidreb>>1)
		{
				conf.tek_gr_kal=1;
				new_group=1;
				 // gr 2
				PORT_L1->BSRRH = PIN_L1;  	// off  PIN_L1
				PORT_L2->BSRRL = PIN_L2;	  // on PIN_L2
				kol_gr1_vkl=0;
				kol_gr2_vkl=0;
		}				
	
	/*
	  RCC_AHB1PeriphClockCmd(GPIO_CLK[Led], ENABLE);


  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
	*/
	
			// проверяем реле на срабатывание
//	test_rele(fz[0], 0);	
	test_rele(ADC3ConvertedValue, 0);	
	
	
	// раз в 10 мс
	// находим среднее значение
	summa[0]+=ADC3ConvertedValue;
	 
//		summa[0]+=100;
	kol_average++;
	
	// раз в 100 мс
	if (kol_average==10)
	{					
		average[0]=summa[0]/kol_average;
		kol_average=0;
		summa[0]=0;
		
		// раз в 100 мс
		// вычисляем физическую величину
		
		if (conf.tek_gr_kal==conf.revers_group_select)
				if (sost_pribl==0)
					fz[0]=fiz_vel(average[0],&conf.gr_kal1.tabl1);
				else
					fz[0]=fiz_vel(average[0],&conf.gr_kal1.tabl2);
		else
				if (sost_pribl==0)
					fz[0]=fiz_vel(average[0],&conf.gr_kal2.tabl1);
				else
					fz[0]=fiz_vel(average[0],&conf.gr_kal2.tabl2);
			
	
	// находим среднее значение по скользящей средней
		fz_average[0]=moving_average(fz[0],0);
				/*
			fz_average[0]++;
			if (fz_average[0]>1050)
				fz_average[0]=0;
	 */
		// time max
		if (time_max>=conf.time_max) {
				max[0]=0;
				time_max=0;  }
		
		// detect max
		if (fz_average[0]>max[0])
			max[0]=fz_average[0];

		
		if (number_buff)
			if (tk_null==1)
				Buf_adc_zap2[por++]=fz_average[0]|0x8000;	
			else
				Buf_adc_zap2[por++]=fz_average[0]&0x7FFF;			
		else
			if (tk_null==1)
				Buf_adc_zap1[por++]=fz_average[0]|0x8000;	
			else
				Buf_adc_zap1[por++]=fz_average[0]&0x7FFF;
		
		if (por==999)
			por=999;

		
//	read_dat_clock();
	del++;
	if (del==10)
	{		
		// раз в секунду
		tek_max_min++;
		if (tek_max_min==60) {
			time_max++;
			tek_max_min=0;     }
	
		del=0;
		tick++;
		
		if (tick==600000)
			tick=0;
		
		time_label=tick;
		
		bufout[0]++; 										// sec
		if 	(bufout[0]>=60)
		{		bufout[0]=0;
				bufout[1]++;								// min
				
				if 	(bufout[1]>=60)
				{
						bufout[1]=0;
						bufout[2]++;						// hour
						if (bufout[2]==24)
						{
							bufout[2]=0;
							bufout[4]++;					// date

							if (bufout[4]==32)
							{
								bufout[4]=1;
								bufout[5]++;				// month
								if (bufout[5]==13)
								{
									bufout[5]=1;
									bufout[6]++;			// year
								}
							}
						}
				}
		}
		

		
//		rtc_Get(&DT1);	
		
	//	if (DT1.Seconds==0)
		if (bufout[0]==0)
		{
			number_buff^=1;
//		DT_zap=DT1;
			
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
			if (DT_zap.Hours!=99)
				buffering=1;
			por=0;
									
			DT_zap.Hours=bufout[2];
			DT_zap.Minutes=bufout[1];
			DT_zap.Seconds=bufout[0];			
		}
		
		 if (tick%2!=0)
			 STM_EVAL_LEDOn(LED3);		 		
		 else	
		 {
			u8 need_read_ds=0;
			 STM_EVAL_LEDOff(LED3);	 
			
			 if ((bufout[0]==50)|(bufout[0]==51))
			//		error_ds=1; 
					need_read_ds=1;
			 
			 if ((error_ds==1)|(need_read_ds==1))
			 {
					u16 i=0;
				 
					need_read_ds=0;
					b_err_cl=0;
					error_ds=0;

					if ((bufout[2]==23)&(bufout[1]==59))
						if ((conf.rez16!=0)&(conf.rez16!=32768))
							kon_sut=1;
					
					if (kon_sut==1)
						read_ds(14);
					else
						read_ds(8);
					
					if ((error_ds==0)&(b_err_cl==0))
					{
						bufout[0]=((zbuf[0]&0x70)>>4)*10+(zbuf[0]&0x0F);		// sec
						bufout[1]=((zbuf[1]&0x70)>>4)*10+(zbuf[1]&0x0F);    // min
						bufout[2]=((zbuf[2]&0x30)>>4)*10+(zbuf[2]&0x0F);    // hour
						
						bufout[4]=((zbuf[4]&0x30)>>4)*10+(zbuf[4]&0x0F);    // day
						bufout[5]=((zbuf[5]&0x10)>>4)*10+(zbuf[5]&0x0F);    // month
						bufout[6]=((zbuf[6]&0xF0)>>4)*10+(zbuf[6]&0x0F);    // year
						
						
						if (kon_sut==1)
						{
								need_read_ds=1;
								bufout[8]=zbuf[9];     // hour
								bufout[9]=zbuf[10];    // day
								bufout[10]=zbuf[11];   // month
								bufout[11]=zbuf[12];   // year
							
						  	// корректировка часов раз в сутки
							  //(еще не корректировали в эти сутки)
								if ((bufout[9]!=bufout[4])|(bufout[10]!=bufout[5])|(bufout[11]!=bufout[6])) 
								{
										if (fl_need_correct_ds==0)
												fl_need_correct_ds=1;
										
										if (conf.rez16>32760)
												kol_time_korr=(conf.rez16-32768);
										else
												kol_time_korr=conf.rez16;  										
								}
								else
								{
										if (bufout[8]!=24)
										{  u32 tmp=0;
											if (fl_need_correct_ds==0)
													fl_need_correct_ds=1;	
											
											if (conf.rez16>32760)
											{
												tmp=(u32) (24-bufout[8])*(conf.rez16-32768)/24;
												kol_time_korr=(u16) tmp;
											}
											else
											{
												tmp=(u32) (24-bufout[8])*conf.rez16/24; 
												kol_time_korr=(u16) tmp; 
											}
										}
										else
										{
											kon_sut=0;	
											need_read_ds=0;	
											/*
											//  temporary
											if (conf.rez16>32760)
												kol_time_korr=(conf.rez16-32768);
											else
												kol_time_korr=conf.rez16; 
											
											if (fl_need_correct_ds==0)
													fl_need_correct_ds=1;	
											*/
										}											
								}
								
								if (fl_need_correct_ds==1) 
								{	
	//								if (((conf.rez16>32760)&(conf.rez16-32768>40))|((conf.rez16>8)&(conf.rez16<32767)))
	//								{
											tmp_sec=date_to_sec(bufout[6],bufout[5],bufout[4],bufout[2],bufout[1],bufout[0]);									
											if (conf.rez16>32760)
												sec_to_date(tmp_sec-kol_time_korr-1);
											else
												sec_to_date(tmp_sec+kol_time_korr-1); 
/*											
									}
									else
									{
											if (conf.rez16>32760)
									//			bufout[0]-=conf.rez16-32768+2;
									//		bufout[0]-=conf.rez16-32768+1;
										bufout[0]-=conf.rez16-32768-1;
											else
									//			bufout[0]+=conf.rez16-2;
										//		bufout[0]+=conf.rez16-1;
												bufout[0]+=conf.rez16+1;
									}
			*/														
									
							//		sec_to_date(tmp_sec-10);   
									fl_need_correct_ds=2;
								}
									
								if (fl_need_correct_ds==2) 
								{
									start_ds();
									write_bait_ds(0xD0);
									write_bait_ds(0x00);

									write_bait_ds((((bufout[0]/10)<<4)+(bufout[0]%10))&0x7F);
									write_bait_ds(((bufout[1]/10)<<4)+(bufout[1]%10));
									write_bait_ds(((bufout[2]/10)<<4)+(bufout[2]%10));			
									write_bait_ds(1);
									write_bait_ds(((bufout[4]/10)<<4)+(bufout[4]%10));		
									write_bait_ds(((bufout[5]/10)<<4)+(bufout[5]%10));
									write_bait_ds(((bufout[6]/10)<<4)+(bufout[6]%10));
									
		
									write_bait_ds(0);  // байт управление
									
									write_bait_ds(conf.tek_gr_kal&0x01);  // группа калибровок
									
									//  время корректировки часов

							 // 	write_bait_ds(bufout[2]);	  // hour	
							//		write_bait_ds(16);		      // hour	
									write_bait_ds(24);		      // hour	
									write_bait_ds(bufout[4]);		// день								
									write_bait_ds(bufout[5]);   // месяц
									write_bait_ds(bufout[6]);		// год 																
	/*
									write_bait_ds(1);		// день								
									write_bait_ds(8);   // месяц
									write_bait_ds(13);		// год 
	*/								
									stop_ds();  
									
									if ((error_ds==0)&(b_err_cl==0))
										fl_need_correct_ds=3;
									else
										error_ds=1;
								}	


								if (fl_need_correct_ds==3)
								{
									// записываем когда откорректировали
									kon_sut=0;
									if ((error_ds==0)&(b_err_cl==0))
									{
										fl_need_correct_ds=0;		
										need_read_ds=0;
									}
									else
										error_ds=1;
								}	
						}
					}
					else
						error_ds=1;
				}
		} 

	// конец раз секунду			
	 }	

//		TIM_Cmd(TIM6, ENABLE);  // start timer for indicators

//	 indicate_lin(1,(u16) fz_average[0], (u16) 1000, (u16) conf.lin.kol_st);	
 update_indicators();
	
 // конец раз в 100 мс
	}	
	
	if (new_group)
	{

			b_err_cl=0;
			error_ds=0;

	
			//		write_dat_clock();
			start_ds();
			write_bait_ds(0xD0);
			write_bait_ds(0x08);
			
			write_bait_ds(conf.tek_gr_kal&0x01);
			
			stop_ds();
			
			if ((error_ds==0)&(b_err_cl==0))
					new_group=0;
		
	}
	
	if (new_komand)
	{
		u16 tmp=0;
	
		// wrt_conf
		if ((RxBuffer[2]=='w')&(RxBuffer[3]=='r')&(RxBuffer[4]=='t')&(RxBuffer[5]=='_')&(RxBuffer[6]=='c')&(RxBuffer[7]=='o')&(RxBuffer[8]=='n')&(RxBuffer[9]=='f'))
		{
			u16 i=0;
			u8 errors=0;
			uint32_t Temp;
			
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);
			GPIO_WriteBit(GPIOD, rx_pin_en, Bit_SET); 
	
			__disable_irq();
	//		sleep(1000);
			FLASH_Unlock();
	//		sleep(1000);
			FLASH_EraseSector(STR_FLASH,VoltageRange_3);
	//		sleep(1000);

			FLASH_Lock();
	//		sleep(1000);
			FLASH_Unlock();
	//		sleep(1000);
			
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
				
				FLASH_ProgramByte(ADDR_FLASH+(i>>1), tmp3);				
				*(__IO uint8_t *) (BKPSRAM_BASE + (i>>1)) = tmp3;//*(__IO uint8_t *) ((__IO uint8_t *) (&conf) + i);
				if (*(__IO uint8_t *) (BKPSRAM_BASE + (i>>1)) != tmp3)
						errors=1;
			}
			

			FLASH_Lock();
	//		sleep(1000);
			__enable_irq();
			
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
	
		// write_dat_clock();
		
			bufout[0]=((RxBuffer[20]-0x30)*10)+(RxBuffer[21]-0x30);		// sec
			bufout[1]=((RxBuffer[18]-0x30)*10)+(RxBuffer[19]-0x30);    // min
			bufout[2]=((RxBuffer[16]-0x30)*10)+(RxBuffer[17]-0x30);    // hour
			
			bufout[4]=((RxBuffer[10]-0x30)*10)+(RxBuffer[11]-0x30);    // day
			bufout[5]=((RxBuffer[12]-0x30)*10)+(RxBuffer[13]-0x30);    // month
			bufout[6]=((RxBuffer[14]-0x30)*10)+(RxBuffer[15]-0x30);    // year


			start_ds();
			write_bait_ds(0xD0);
			write_bait_ds(0x00);
			
			write_bait_ds(((RxBuffer[20]-0x30)<<4)+(RxBuffer[21]-0x30));
			write_bait_ds(((RxBuffer[18]-0x30)<<4)+(RxBuffer[19]-0x30));
			write_bait_ds(((RxBuffer[16]-0x30)<<4)+(RxBuffer[17]-0x30));			
			write_bait_ds(1);
			write_bait_ds(((RxBuffer[10]-0x30)<<4)+(RxBuffer[11]-0x30));		
			write_bait_ds(((RxBuffer[12]-0x30)<<4)+(RxBuffer[13]-0x30));
			write_bait_ds((((RxBuffer[14]-0x30)<<4))+(RxBuffer[15]-0x30));
				
			
			write_bait_ds(0);  // байт управление
			
			write_bait_ds(conf.tek_gr_kal&0x01);  // группа калибровок
			
			//  время корректировки часов

			write_bait_ds(bufout[2]);		// hour	
			write_bait_ds(bufout[4]);		// день								
			write_bait_ds(bufout[5]);   // месяц
			write_bait_ds(bufout[6]);		// год 	
	
/*
      // temporary
			write_bait_ds(18);		// hour	
			write_bait_ds(1);		// день								
			write_bait_ds(8);   // месяц
			write_bait_ds(13);		// год 
	*/		
			stop_ds();
				

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
/*
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
*/
			// date
			TxBuffer[5]=(uint8_t)(bufout[6]/10)+(uint8_t)0x30;	
			TxBuffer[6]=(uint8_t)(bufout[6]%10)+(uint8_t)0x30;	
			TxBuffer[7]=(uint8_t)(bufout[5]/10)+(uint8_t)0x30;	
			TxBuffer[8]=(uint8_t)(bufout[5]%10)+(uint8_t)0x30;	
			TxBuffer[9]=(uint8_t)(bufout[4]/10)+(uint8_t)0x30;	
			TxBuffer[10]=(uint8_t)(bufout[4]%10)+(uint8_t)0x30;	
			TxBuffer[11]=0x20;				
			
			// time
			TxBuffer[12]=(uint8_t)(bufout[2]/10)+(uint8_t)0x30;	
			TxBuffer[13]=(uint8_t)(bufout[2]%10)+(uint8_t)0x30;			
			TxBuffer[14]=(uint8_t)(bufout[1]/10)+(uint8_t)0x30;	
			TxBuffer[15]=(uint8_t)(bufout[1]%10)+(uint8_t)0x30;	
			TxBuffer[16]=(uint8_t)(bufout[0]/10)+(uint8_t)0x30;	
			TxBuffer[17]=(uint8_t)(bufout[0]%10)+(uint8_t)0x30;	
			TxBuffer[18]=0x20;
			
/*
			// date
			tmp=bufout[6];			// year
			TxBuffer[5]=(uint8_t)((tmp>>4)+(uint8_t)0x30);	
			TxBuffer[6]=(uint8_t)((tmp&0x0F)+(uint8_t)0x30);	
			
			tmp=(bufout[5]);   // month
			TxBuffer[7]=(uint8_t)(((tmp&0x10)>>4)+(uint8_t)0x30);	
			TxBuffer[8]=(uint8_t)((tmp&0x0F)+(uint8_t)0x30);	
			
			tmp=(bufout[4]);		// date
			TxBuffer[9]=(uint8_t)(((tmp&0x30)>>4)+(uint8_t)0x30);	
			TxBuffer[10]=(uint8_t)((tmp&0x0F)+(uint8_t)0x30);	
			TxBuffer[11]=0x20;				
			
			// time
			tmp=(bufout[2]);		//	hours
			TxBuffer[12]=(uint8_t)(((tmp&0x70)>>4)+(uint8_t)0x30);	
			TxBuffer[13]=(uint8_t)((tmp&0x0F)+(uint8_t)0x30);	
				
			tmp=(bufout[1]);		//	minute
			TxBuffer[14]=(uint8_t)(((tmp&0x70)>>4)+(uint8_t)0x30);	
			TxBuffer[15]=(uint8_t)((tmp&0x0F)+(uint8_t)0x30);	
			
			tmp=(bufout[0]);		//	seconds
			TxBuffer[16]=(uint8_t)(((tmp&0x70)>>4)+(uint8_t)0x30);	
			TxBuffer[17]=(uint8_t)((tmp&0x0F)+(uint8_t)0x30);	
			TxBuffer[18]=0x20;
*/			
			// dat pribl
			TxBuffer[19]=sost_pribl+0x30;
			TxBuffer[20]=0x20;
			
			
			// zn from adc with calibr and averaging
			tmp=ADC3ConvertedValue%10000;
			TxBuffer[21]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
			tmp%=1000;
			TxBuffer[22]=(uint8_t)(tmp/100)+(uint8_t)0x30;
			tmp%=100;		
			TxBuffer[23]=(uint8_t)(tmp/10)+(uint8_t)0x30;
			tmp%=10;	
			TxBuffer[24]=(uint8_t)(tmp)+(uint8_t)0x30;
			TxBuffer[25]=0x20;		
			
			tmp=fz_average[0]%10000;
			TxBuffer[26]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
			tmp%=1000;
			TxBuffer[27]=(uint8_t)(tmp/100)+(uint8_t)0x30;
			tmp%=100;		
			TxBuffer[28]=(uint8_t)(tmp/10)+(uint8_t)0x30;
			tmp%=10;	
			TxBuffer[29]=(uint8_t)(tmp)+(uint8_t)0x30;		
			TxBuffer[30]=0x20;
			
			tmp=max[0]%10000;
			TxBuffer[31]=(uint8_t)(tmp/1000)+(uint8_t)0x30;
			tmp%=1000;
			TxBuffer[32]=(uint8_t)(tmp/100)+(uint8_t)0x30;
			tmp%=100;		
			TxBuffer[33]=(uint8_t)(tmp/10)+(uint8_t)0x30;
			tmp%=10;	
			TxBuffer[34]=(uint8_t)(tmp)+(uint8_t)0x30;		

			TxBuffer[35]=0x20;
			TxBuffer[36]=(uint8_t)(avariya)+(uint8_t)0x30;	

			TxBuffer[37]=0x20;
			TxBuffer[38]=(uint8_t)(sost_flesh)+(uint8_t)0x30;
			
			TxBuffer[39]=0x20;
			TxBuffer[40]=(uint8_t)(conf.tek_gr_kal)+(uint8_t)0x30;
			
			TxBuffer[41]=0x20;
			TxBuffer[42]=(uint8_t)(error_ds)+(uint8_t)0x30;
		/*
     + need:
			1 avariya   
			2 zapis norm or error  
			3 tek gr kal
*/
			txsize=43;
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
				TxBuffer[i]=(*(__IO uint8_t *) (BKPSRAM_BASE + i));
			}	
	
			for (i = 0; i < (txsize); i += 4)
			{
				uint32_t data;				
				data= *(__IO uint32_t*) (ADDR_FLASH+i);
			
				TxBuffer[i+3]=(u8) (data>>24);
				TxBuffer[i+2]=(u8) (data>>16);
				TxBuffer[i+1]=(u8) (data>>8);
				TxBuffer[i]=(u8) data;
			}	

//}

			tekper=0;
			USART_SendData(USART2, 0x3A);
		}		
		
		new_komand=0;
	}
	
		//	indicators[0].chislo=1234;
		//		indicators[1].chislo=1234;
		//		indicate (0);
		//		indicate (1);
			
	PORT_Conrtol->BSRRH = PIN_Conrtol;  	// off  PIN_Control
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
