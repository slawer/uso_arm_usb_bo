//
// rtc.c STM32F4Discovery
//

#include "rtc.h"
#include <stm32f4xx.h>

#include <string.h>

// Выключить защиту от записи
static void rtc_Unlock(void)
{
    // Запишем эти значения по очереди
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
}

// Включить защиту от записи
static void rtc_Lock(void)
{
    // Запишем какую-нибудь фигню, главное, чтоб не правильную
    RTC->WPR = 0xFF;
}

// Установить дату
static void rtc_SetDate(uint8_t Day, uint8_t Month, uint8_t Year, uint8_t DayOfWeek)
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
static void rtc_SetTime(uint8_t Hours, uint8_t Minutes, uint8_t Seconds)
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
    // Если часы запущены, делать тут нечего.
    if(RTC->ISR & RTC_ISR_INITS) return;
    
    // Включим тактирование PWR
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    // Разрешим доступ к управляющим регистрам энергонезависимого домена
    PWR->CR |= PWR_CR_DBP;
    
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
    
    // Снимем защиту от записи с регистров RTC
    rtc_Unlock();
    {
        // Здесь можем менять регистры RTC

        // Войдём в режим инициализации:
        RTC->ISR |= RTC_ISR_INIT;
        
        // Ждём, когда это произойдёт
        while(!(RTC->ISR & RTC_ISR_INITF)) {}
        
        // Часы остановлены. Режим инициализации
        // Настроим предделитель для получения частоты 1 Гц.
        
        // LSI: 
        // LSE: нужно разделить на 0x7fff (кварцы так точно рассчитаны на это)
        {  //  32768Hz, а нам нужны
            uint32_t Sync = 249;   // 15 бит
            uint32_t Async =127;  // 7 бит
            
            // Сначала записываем величину для синхронного предделителя
            RTC->PRER = Sync;
            
            // Теперь добавим для асинхронного предделителя
            RTC->PRER =Sync | (Async << 16);
					
			//		RTC->PRER = 0x00000000; // RESET PRER register
			//		RTC->PRER |= (0xFF<<0); // 255 + 1 Synchronous prescaler factor set
			//		RTC->PRER |= (0x7F<<16); // 127 + 1 Asynchronous prescaler factor set
        }
        
        // Устанавливаем дату: 30.05.13, пятница
        rtc_SetDate(2, 6, 13, 7);
        
        // Устанавливаем время: 15:00:00
        rtc_SetTime(15, 15, 00);
        
        // Переведём часы в 24-часовой формат
        RTC->CR |= RTC_CR_FMT;
        
        // Инициализация закончилась
        RTC->ISR &= ~RTC_ISR_INIT;
    }   
    rtc_Lock();
    
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
