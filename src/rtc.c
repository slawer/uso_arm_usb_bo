//
// rtc.c STM32F4Discovery
//

#include "rtc.h"
#include <stm32f4xx.h>

#include <string.h>

#include "my.h"


u16 bkp=0;
u16 size=0;

// Выключить защиту от записи
void rtc_Unlock(void)
{
    // Запишем эти значения по очереди
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
}

// Включить защиту от записи
 void rtc_Lock(void)
{
    // Запишем какую-нибудь фигню, главное, чтоб не правильную
    RTC->WPR = 0xFF;
}

// Установить дату
void rtc_SetDate(uint8_t Day, uint8_t Month, uint8_t Year, uint8_t DayOfWeek)
{
    uint32_t Tens, Units;
    uint32_t TempReg = 0;
    
    // Очистим поле даты
    TempReg = 0;
    
    // Запишем год
    {
        Tens  = (Year / 10) & 0x0f;          // Десятки лет
        Units = (Year - (Tens * 10)) & 0x0f; // Единицы лет
        
        TempReg |= (Tens  << 20); // YT, 20
        TempReg |= (Units << 16); // YU, 16
    }
    // Запишем месяц
    {
        Tens  = (Month / 10) & 0x01;          // Десятки месяцев
        Units = (Month - (Tens * 10)) & 0x0f; // Единицы месяцев
        
        TempReg |= (Tens  << 12); // MT, 12
        TempReg |= (Units << 8);  // MU, 8
    }
    // Запишем день
    {
        Tens  = (Day / 10) & 0x03;          // Десятки дней
        Units = (Day - (Tens * 10)) & 0x0f; // Единицы дней
        
        TempReg |= (Tens  << 4); // DT, 4
        TempReg |= (Units << 0);  // DU, 0
    }
    // День недели:
    {
        TempReg |= ((DayOfWeek & 0x07) << 13); // WDU, 13
    }
    
    // Записывать надо всё сразу
    RTC->DR = TempReg;
}

// Установить время
//static void rtc_SetTime(uint8_t Hours, uint8_t Minutes, uint8_t Seconds)
void rtc_SetTime(uint8_t Hours, uint8_t Minutes, uint8_t Seconds)
{
    uint32_t Tens, Units;
    uint32_t TempReg = 0;
    
    // Очистим поле даты
    TempReg = 0;
    
    // Запишем часы
    {
        Tens  = (Hours / 10) & 0x03;          // Десятки часов
        Units = (Hours - (Tens * 10)) & 0x0f; // Единицы часов
        
        TempReg |= (Tens  << 20); // HT, 20
        TempReg |= (Units << 16); // HU, 16
    }
    // Запишем минуты
    {
        Tens  = (Minutes / 10) & 0x07;          // Десятки минут
        Units = (Minutes - (Tens * 10)) & 0x0f; // Единицы минут
        
        TempReg |= (Tens  << 12); // MNT, 12
        TempReg |= (Units << 8);  // MNU, 8
    }
    // Запишем секунды
    {
        Tens  = (Seconds / 10) & 0x07;          // Десятки секунд
        Units = (Seconds - (Tens * 10)) & 0x0f; // Единицы секунд
        
        TempReg |= (Tens  << 4); // ST, 4
        TempReg |= (Units << 0);  // SU, 0
    }
    
    // Записывать надо всё сразу
    RTC->TR = TempReg;
}

// Сброс состояния часов
void rtc_Reset(void)
{
    // Включим тактирование PWR
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    // Разрешим доступ к управляющим регистрам энергонезависимого домена
    PWR->CR |= PWR_CR_DBP;
    
    // Выберем его как источник тактирования RTC:
    RCC->BDCR |=  RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
}

// Инициализация модуля
void rtc_Init(void)
{
		u32 tmp=0;
	
	  GPIO_InitTypeDef      GPIO_InitStructure;
	
    // Если часы запущены, делать тут нечего.
    if(RTC->ISR & RTC_ISR_INITS) return;
    
    // Включим тактирование PWR
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    // Разрешим доступ к управляющим регистрам энергонезависимого домена
    PWR->CR |= PWR_CR_DBP;
   
/*
	//  Кварц MC306-G-06Q-32.768 (JFVNY)
	 //  start LSE
		RCC->BDCR |= RCC_BDCR_LSEON;
		while ((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON) {}
		
		RCC->BDCR |=  RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
		
	// Выберем его как источник тактирования RTC:
    RCC->BDCR &= ~RCC_BDCR_RTCSEL; // сбросим
    RCC->BDCR |= (RCC_BDCR_RTCSEL_0); // запишем 0b10
		
		   // Включим тактирование RTC
    RCC->BDCR |= RCC_BDCR_RTCEN;
*/
 
/* RTC configuration set 
PWR->CR |= (1<<8);	 // Access to RTC and RTC backup registers and backup SRAM enabled
RCC->CSR &= (1<<0);	 // LSI Off
RCC->BDCR = 0x00000000;	 // Reset BDCR register
RCC->BDCR |= (1<<15);	 // RTC clock enable
RCC->BDCR |= (1<<0);	 // LSE On
RCC->BDCR &= ~(1<<2);	 // LSE not bypassed quartz On
RCC->BDCR &= ~(1<<16);	 // Backup domain software reset not activated
RCC->BDCR |= (0x1<<8);	 // LSE used as the RTC clock
RTC->WPR = 0x000000CA;	 // Key protect 1
RTC->WPR = 0x00000053;	 // Key protect 2
RTC->ISR |= (1<<7);	 // Initialization mode On
for(;((RTC->ISR & 0x40) == 0x00);)	// delay while initialization flag will be set
{
}
*/	
	/* 
	JFVNY MC306G06 6 50000 32768 2 9.3
	PC14/OSC32_IN   (PC14)
	PC15/OSC32_OUT (PC15)
	PB2/BOOT1    (PB2)
*/


/*
	//     push pull out  in kvarc
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14|GPIO_Pin_15;      		  //  vvod  knopka 1
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;    										// 	rezim vivoda
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 									//	speed
	GPIO_Init(GPIOC, &GPIO_InitStructure); 

	for(tmp=0;tmp<10000;tmp++)	// delay while initialization flag will be set
	{ __ASM volatile ("nop"); 	}

//	GPIOA->BSRR = GPIO_BSRR_BS14; // Boot the cristal 
	GPIOA->BSRRL = GPIO_Pin_14;
//	GPIOA->BSRRL = GPIO_Pin_15;
	GPIOB->BSRRL = GPIO_Pin_2;
	
	for(tmp=0;tmp<1000000;tmp++)	
	{ __ASM volatile ("nop"); 	}
	
//     analog in kvarc
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14|GPIO_Pin_15;      		  //  vvod  knopka 1
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;    										// 	rezim vivoda
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  										//GPIO_OType_PP		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 									//	speed
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
*/

 /*
		
		RCC->BDCR |=  RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
		
	// Выберем его как источник тактирования RTC:
    RCC->BDCR &= ~RCC_BDCR_RTCSEL; // сбросим
    RCC->BDCR |= (RCC_BDCR_RTCSEL_0); // запишем 0b10
		
		   // Включим тактирование RTC
    RCC->BDCR |= RCC_BDCR_RTCEN;
	*/	
/*	

Попробуйте дать ему пинка. Сконфигурируйте ноги на выход и подайте напряжение на кварц на четверть периода. Потом уже снова на вход и запускайте генератор. Заводились даже 12 пФ кварцы. Только работали нестабильно.
	GPIOC->CRH |= GPIO_CRH_MODE14_1 |GPIO_CRH_MODE15_1; // Out 2 MHZ
	GPIOC->CRH &= ~(GPIO_CRH_CNF14|GPIO_CRH_CNF15); // PUSH pull
	Delay();
	GPIOA->BSRR = GPIO_BSRR_BS14; // Boot the cristal 
	Delay();
	GPIOC->CRH &= ~(GPIO_CRH_MODE14_1|GPIO_CRH_MODE15_1); // Analog IN 
	RCC->BDCR |= RCC_BDCR_LSEON;

KX-38 32.768 kHz 6pF,
		32.768kHz crystal, with 6pF CL and 50kOhm ESR. 
		a 15MOhm parallel resistor.
if (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
{

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // Enable PWR clock 
 
 //   PWR_BackupAccessCmd(ENABLE); // Allow access to BKP Domain 
		 RCC->CSR = RCC_CSR_LSION;
    RCC_LSEConfig(RCC_LSE_ON); // Enable the LSE oscillator 
 
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) // Wait till LSE is ready
		{
		 __ASM volatile ("nop");   
		}
}
*/
/*
    // Запускаем LSI:
    RCC->CSR |= RCC_CSR_LSION;
    
    // Ждём, когда он заведётся
    while(!(RCC->CSR & RCC_CSR_LSIRDY)) {}
    
    // Ок, генератор на 32 кГц завёлся.
    
    // Сбросим состояние энергонезависимого домена
    RCC->BDCR |=  RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    
    // Выберем его как источник тактирования RTC:
    RCC->BDCR &= ~RCC_BDCR_RTCSEL; // сбросим
    RCC->BDCR |= (RCC_BDCR_RTCSEL_1); // запишем 0b10
      
    // Включим тактирование RTC
    RCC->BDCR |= RCC_BDCR_RTCEN;
 */

/*
● Access to the backup SRAM
1. Enable the power interface clock by setting the PWREN bits in the RCC APB1
peripheral clock enable register (RCC_APB1ENR)
2. Set the DBP bit in the PWR power control register (PWR_CR) to enable access to the
backup domain
3. Enable the backup SRAM clock by setting BKPSRAMEN bit in the RCC AHB1
peripheral clock register (RCC_AHB1ENR)

uint8_t *BKPRam = (uint8_t *)0x40024000;


*/	

   // Запускаем LSI:
    RCC->CSR |= RCC_CSR_LSION;
    
    // Ждём, когда он заведётся
    while(!(RCC->CSR & RCC_CSR_LSIRDY)) {}
    
    // Ок, генератор на 32 кГц завёлся.
    
    // Сбросим состояние энергонезависимого домена
    RCC->BDCR |=  RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    
    // Выберем его как источник тактирования RTC:
    RCC->BDCR &= ~RCC_BDCR_RTCSEL; // сбросим
    RCC->BDCR |= (RCC_BDCR_RTCSEL_1); // запишем 0b10
      
    // Включим тактирование RTC
    RCC->BDCR |= RCC_BDCR_RTCEN;
			
/*
		 //  start LSE
		 RCC->BDCR |= RCC_BDCR_LSEON;
		 while ((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON) {}			
		 while (!(RCC->BDCR & RCC_BDCR_LSERDY)) {}			
		 RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_0;
			
      // Снимем защиту от записи с регистров RTC
      rtc_Unlock();
        // Войдём в режим инициализации:
      RTC->ISR |= RTC_ISR_INIT;
			while(!(RTC->ISR & RTC_ISR_INITF)) {}

					RTC->PRER = 0x00000000; // RESET PRER register
					RTC->PRER |= (0xFF<<0); // 255 + 1 Synchronous prescaler factor set
					RTC->PRER |= (0x7F<<16); // 127 + 1 Asynchronous prescaler factor set
     */          
        // Устанавливаем дату: 30.05.13, пятница
				//       rtc_SetDate(2, 6, 13, 7);       
        // Устанавливаем время: 15:00:00
				//      rtc_SetTime(0, 0, 00);       
        // Переведём часы в 24-часовой формат
        RTC->CR |= RTC_CR_FMT;        
        // Инициализация закончилась
        RTC->ISR &= ~RTC_ISR_INIT;   
    rtc_Lock();
		
		// Allow access to BKP Domain 
//PWR_BackupAccessCmd(ENABLE);

// Write to the first RTC Backup Data Register 
RTC_WriteBackupRegister(RTC_BKP_DR2,0xA5A5);
		
		// Backup SRAM **************************************************************
//Enable BKPRAM Clock 
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);

//Enable the Backup SRAM low power Regulator to retain it's content in VBAT mode 
//PWR_BackupRegulatorCmd(ENABLE);

// Wait until the Backup SRAM low power Regulator is ready 
//while(PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET)
{
}

 {
	 
	 extern st_conf conf;
	 u32 i=0,i1=0, errorindex=0;
	 
   bkp=RTC_ReadBackupRegister(RTC_BKP_DR2);


//  Backup SRAM ************************************************************
  // Enable BKPRAM Clock 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);

/*
  // Write to Backup SRAM with 32-Bit Data 
  for (i = 0; i < 0x1000; i += 4)
  {
    *(__IO uint32_t *) (BKPSRAM_BASE + i) = i;
  }
*/
//	sizeof
  // read config rrom backup SRAM
	 
	 

	/*
  for (i = 0; i < size; i += 2)
  {
		(*(__IO uint16_t *) ((__IO uint16_t *) (&conf) + i))=(u16)i; //(*(__IO uint32_t *) (BKPSRAM_BASE + i));
	}	
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
	
	conf.indicators[0].numb=15;
	conf.indicators[0].kol_cifr=16;
	conf.indicators[1].numb=17;
	conf.indicators[1].kol_cifr=18;
	conf.indicators[2].numb=19;
	conf.indicators[2].kol_cifr=20;
	conf.indicators[3].numb=21;
	conf.indicators[3].kol_cifr=2;
	conf.gr_kal1.tabl1.kod[0]=23;
	conf.gr_kal1.tabl1.fz[0]=24;
	conf.gr_kal1.tabl2.kod[0]=25;
	conf.gr_kal1.tabl2.fz[0]=26;
	conf.gr_kal2.tabl1.kod[0]=27;
	conf.gr_kal2.tabl1.fz[0]=28;
	conf.gr_kal2.tabl2.kod[0]=29;
	conf.gr_kal2.tabl2.fz[0]=30;
	conf.gr_kal2.tabl2.kod[9]=31;
	conf.gr_kal2.tabl2.fz[9]=32;
*/
/*
uint32_t flash_read(uint32_t address) {
return (*(__IO uint32_t*) address);
}
*/
	size=sizeof(st_conf);
	
	for (i = 0; i < size; i += 1)
  {
		(*(__IO uint8_t *) ((__IO uint8_t *) (&conf) + i))=(*(__IO uint8_t *) (ADDR_FLASH + i));
	}	
	
	size=sizeof(st_conf);
	
	for (i = 0; i < size; i += 1)
  {
		(*(__IO uint8_t *) ((__IO uint8_t *) (&conf) + i))=(*(__IO uint8_t *) (BKPSRAM_BASE + i));
	}	
	
		kol_usr=conf.per_usr;
		conf.tm_antidreb=conf.tm_antidreb*10;
	
		if (conf.tm_antidreb==0)
				conf.tm_antidreb=1;

//	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 
	GPIO_InitStructure.GPIO_Pin   = PIN_RELE;      		//  vivod RELE
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;    // rezim vivoda
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//  may be PP - ???
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //speed
	GPIO_Init(PORT_RELE, &GPIO_InitStructure); 


	GPIO_InitStructure.GPIO_Pin   = PIN_K1;      		  //  vvod  knopka 1
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;    // 	rezim vivoda
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //	speed
	GPIO_Init(PORT_K1, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin   = PIN_K2;      		  //  vvod  knopka 2
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;    // 	rezim vivoda
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //	speed
	GPIO_Init(PORT_K2, &GPIO_InitStructure); 
	
	if (conf.tek_gr_kal==0)
	{
		//  dr kal ==0 - pervaya
		PORT_L1->BSRRL = PIN_L1;  // on  PIN_L1
	  PORT_L2->BSRRH = PIN_L2;	// off PIN_L2
	}
	else
	{
		//  dr kal ==1 - vtoraya
		PORT_L1->BSRRH = PIN_L1;  // off  PIN_L1
	  PORT_L2->BSRRL = PIN_L2;	// on   PIN_L2
	}
      
		sost_flesh=0;	
		PORT_ZAP_EN->BSRRH = PIN_ZAP_EN;  // off  PORT_ZAP_EN
		PORT_ZAP_DIS->BSRRL = PIN_ZAP_DIS;  // on  PORT_ZAP_DIS
		
		avariya=0;
		PORT_AVARIYA->BSRRH = PIN_AVARIYA;   
	
		PORT_RELE->BSRRH = PIN_RELE;	// on PIN_RELE	
	
	/*
		conf.por_rele
	  conf.time_max
	  conf.tm_antidreb
		conf.tm_rele_off
		conf.tm_rele_on
		conf.
	*/	
	
	/*
	for (i = 0; i < size; i += 1)
  {
    *(__IO uint8_t *) (BKPSRAM_BASE + i) = 0;
  }
	*/
	
	/*
	for (i = 0; i < size; i += 1)
  {
    *(__IO uint8_t *) (BKPSRAM_BASE + i) = *(__IO uint8_t *) ((__IO uint8_t *) (&conf) + i);
  }
	*/
	
	
/*
	conf.address=255;
	conf.ver_po_st=255;
	conf.ver_po_ml=255;
	conf.per_usr=255;
	conf.time_max=255;
	conf.tek_gr_kal=255;
	conf.gr_kal1.tabl1.fz[0]=255;
	conf.gr_kal1.tabl1.kod[0]=255;
	conf.gr_kal1.tabl2.fz[0]=255;
	conf.gr_kal1.tabl2.kod[0]=255;
	
	conf.gr_kal2.tabl1.fz[0]=255;
	conf.gr_kal2.tabl1.kod[0]=255;
	conf.gr_kal2.tabl2.fz[0]=255;
	conf.gr_kal2.tabl2.kod[0]=255;
	conf.revers_group_select=255;
	conf.revers_peredacha_select=255;
	conf.tm_antidreb=255;
	conf.por_rele=255;
	conf.tm_rele_on=255;
	conf.tm_rele_off=255;
	conf.indicators[0].numb=255;
	conf.indicators[0].kol_cifr=255;
	conf.indicators[1].numb=255;
	conf.indicators[1].kol_cifr=255;
	conf.indicators[2].numb=255;
	conf.indicators[2].kol_cifr=255;
	conf.indicators[3].numb=255;
	conf.indicators[3].kol_cifr=255;
	*/
	/*
	// test read config to comp
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
	
	conf.indicators[0].numb=15;
	conf.indicators[0].kol_cifr=16;
	conf.indicators[1].numb=17;
	conf.indicators[1].kol_cifr=18;
	conf.indicators[2].numb=19;
	conf.indicators[2].kol_cifr=20;
	conf.indicators[3].numb=21;
	conf.indicators[3].kol_cifr=2;
	conf.gr_kal1.tabl1.kod[0]=23;
	conf.gr_kal1.tabl1.fz[0]=24;
	conf.gr_kal1.tabl2.kod[0]=25;
	conf.gr_kal1.tabl2.fz[0]=26;
	conf.gr_kal2.tabl1.kod[0]=27;
	conf.gr_kal2.tabl1.fz[0]=28;
	conf.gr_kal2.tabl2.kod[0]=29;
	conf.gr_kal2.tabl2.fz[0]=30;
	conf.gr_kal2.tabl2.kod[9]=31;
	conf.gr_kal2.tabl2.fz[9]=32;
	
*/
	
}
	
__ASM volatile ("nop");

    
    // Всё, часы запустились и считают время.
		/*
		
		/разрешить тактирование модулей управления питанием и управлением резервной областью
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
  //разрешить доступ к области резервных данных
  PWR->CR |= PWR_CR_DBP;
  //если часы выключены - инициализировать их
  if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
  {
    //выполнить сброс области резервных данных
    RCC->BDCR |=  RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
 
    //выбрать источником тактовых импульсов внешний кварц 32768 и подать тактирование
    RCC->BDCR |=  RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE;
 
    RTC->CRL  |=  RTC_CRL_CNF;
    RTC->PRLL  = 0x7FFF;         //регистр деления на 32768
    RTC->CRL  &=  ~RTC_CRL_CNF;
 
    //установить бит разрешения работы и дождаться установки бита готовности
    RCC->BDCR |= RCC_BDCR_LSEON;
    while ((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON){}
 
    RTC->CRL &= (uint16_t)~RTC_CRL_RSF;
    while((RTC->CRL & RTC_CRL_RSF) != RTC_CRL_RSF){}
 
    return 1;
  }
  return 0;
	*/
	
	
	/*
	
	// one more init кес
	
	RTC_InitTypeDef RTC_InitStructure;
RTC_TimeTypeDef RTC_TimeStructure;
RTC_DateTypeDef RTC_DateStructure;
__IO uint32_t AsynchPrediv = 0, SynchPrediv = 0;
__IO uint32_t TimeDisplay = 0;


		// Enable the PWR APB1 Clock Interface 
RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

//Allow access to BKP Domain 
PWR_BackupAccessCmd(ENABLE);
  
if (RTC_ReadBackupRegister(RTC_BKP_DR2) != 0xA5A5) {

//Enable the PWR clock 
RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

// Allow access to RTC 
PWR_BackupAccessCmd(ENABLE);

#if defined (RTC_CLOCK_SOURCE_LSI) // LSI used as RTC source clock
// The RTC Clock may varies due to LSI frequency dispersion. 
// Enable the LSI OSC 
RCC_LSICmd(ENABLE);

// Wait till LSI is ready 
while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
{
}

// Select the RTC Clock Source 
RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

SynchPrediv = 0xFF;
AsynchPrediv = 0x7F;

#elif defined (RTC_CLOCK_SOURCE_LSE) // LSE used as RTC source clock 
// Enable the LSE OSC 
RCC_LSEConfig(RCC_LSE_ON);

// Wait till LSE is ready 
while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
{
}

// Select the RTC Clock Source 
RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

SynchPrediv = 0xFF;
AsynchPrediv = 0x7F;

#else
#error Please select the RTC Clock source inside the main.c file
#endif // RTC_CLOCK_SOURCE_LSI 

// Enable the RTC Clock 
RCC_RTCCLKCmd(ENABLE);

// Wait for RTC APB registers synchronisation 
RTC_WaitForSynchro();

// Allow access to BKP Domain 
PWR_BackupAccessCmd(ENABLE);

// Write to the first RTC Backup Data Register 
RTC_WriteBackupRegister(RTC_BKP_DR2,0xA5A5);

//Set the Time 
RTC_TimeStructure.RTC_Hours = 22;
RTC_TimeStructure.RTC_Minutes = 11;
RTC_TimeStructure.RTC_Seconds = 00;

// Set the Date 
RTC_DateStructure.RTC_Month = 4;
RTC_DateStructure.RTC_Date = 29;
RTC_DateStructure.RTC_Year = 11;
RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Friday;

//Calendar Configuration 
RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
RTC_InitStructure.RTC_SynchPrediv = SynchPrediv;
RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
RTC_Init(&RTC_InitStructure);

// Set Current Time and Date 
RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
#if 0
//Configure the RTC Wakeup Clock source and Counter (Wakeup event each 1 second) 
RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
RTC_SetWakeUpCounter(0x7FF);

// Enable the Wakeup Interrupt 
RTC_ITConfig(RTC_IT_WUT, ENABLE);

//Enable Wakeup Counter 
RTC_WakeUpCmd(ENABLE);
#endif
// Backup SRAM **************************************************************
//Enable BKPRAM Clock 
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);

//Enable the Backup SRAM low power Regulator to retain it's content in VBAT mode 
PWR_BackupRegulatorCmd(ENABLE);

// Wait until the Backup SRAM low power Regulator is ready 
while(PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET)
{
}

}
else{

// Enable the PWR clock 
RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

// Allow access to RTC 
PWR_BackupAccessCmd(ENABLE);

// Wait for RTC APB registers synchronisation 
RTC_WaitForSynchro();
// Clear the Wakeup Interrupt 
RTC_ClearITPendingBit(RTC_IT_WUT);

// Backup SRAM **************************************************************
// Enable BKPSRAM Clock 
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
}


	
	*/
	
}

// Получить текущее время
void rtc_Get(TDateTime * DateTime)
{
    uint32_t Date = RTC->DR;
    uint32_t Time = RTC->TR;
    
    // Очистим
    memset(DateTime, 0, sizeof(*DateTime));
    
    // Год
    DateTime->Year      = ((Date >> 20) & 0x0f) * 10 + ((Date >> 16) & 0x0f);
    // Месяц
    DateTime->Month     = ((Date >> 12) & 0x01) * 10 + ((Date >>  8) & 0x0f);
    // День
    DateTime->Day       = ((Date >>  4) & 0x03) * 10 + ((Date >>  0) & 0x0f);
    // День недели
    DateTime->DayOfWeek = ((Date >> 13) & 0x07);
    
    // Час
    DateTime->Hours     = ((Time >> 20) & 0x03) * 10 + ((Time >> 16) & 0x0f);
    // Минуты
    DateTime->Minutes   = ((Time >> 12) & 0x07) * 10 + ((Time >> 8) & 0x0f);
    // Секунды
    DateTime->Seconds   = ((Time >> 4) & 0x07) * 10 + ((Time >> 0) & 0x0f);
}
