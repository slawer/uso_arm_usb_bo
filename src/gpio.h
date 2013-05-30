// ***********************************************************
//	gpio.h
// 	Порты ввода-вывода
//
// ***********************************************************

#include <stdint.h>
#include <stdbool.h>

#ifndef _GPIO_H
#define _GPIO_H

    typedef enum { PORTA, PORTB, PORTC, PORTD, PORTE, PORTF, PORTG, PORTH, PORTI } TPort;

    // Инициализация
    void gpio_Init(void);
    
    // Настроить вывод на выход
    void gpio_DigitalOutput(TPort Port, uint8_t Pin);

    // Настроить вывод на вход
    void gpio_DigitalInput(TPort Port, uint8_t Pin);

    // Высокий уровень на ножке
    void gpio_HighLevel(TPort Port, uint8_t Pin);

    // Низкий уровень на ножке
    void gpio_LowLevel(TPort Port, uint8_t Pin);

    // Значение на ножке
    bool gpio_Value(TPort Port, uint8_t Pin);

    // Включить Open Drain (иначе двухтактный)
    void gpio_OpenDrain(TPort Port, uint8_t Pin, bool Enable);
    
    // Установить для вывода альтернативную функцию (0-3)
    void gpio_SetAlternateFunction(uint8_t Port, uint8_t Pin, uint8_t Function);

#endif

