//
// rtc.h
//

#include <stdint.h>
#include <stdbool.h>

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
