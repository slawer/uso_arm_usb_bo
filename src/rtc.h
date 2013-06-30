//
// rtc.h
//

#include <stdint.h>
#include <stdbool.h>

#define ADDR_FLASH  	 0x080E0000 //0x08000000 //0x0800C000 //0x801FC00
#define STR_FLASH  	   FLASH_Sector_11 //0x0800C000 //0x801FC00

//		8000000									 0x100000


#ifndef _RTC_H
#define _RTC_H

    typedef struct
    {
        uint8_t Year;      // Год
        uint8_t Month;     // Месяц
        uint8_t Day;       // День месяца
        
        uint8_t DayOfWeek; // День недели
        
        uint8_t Hours;     // Часы
        uint8_t Minutes;   // Минуты
        uint8_t Seconds;   // Секунды
    } TDateTime;

    // Инициализация модуля
    void rtc_Init(void);

    // Сброс состояния часов
    void rtc_Reset(void);
    
    // Получить текущее время
    void rtc_Get(TDateTime * DateTime);
		
		 void rtc_SetTime(uint8_t Hours, uint8_t Minutes, uint8_t Seconds);
		
		 void rtc_SetDate(uint8_t Day, uint8_t Month, uint8_t Year, uint8_t DayOfWeek);
		
		 void rtc_Unlock(void);
		 void rtc_Lock(void);

#endif
