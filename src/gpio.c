// ***********************************************************
//	gpio.c
// 	Порты ввода-вывода
//
// ***********************************************************

#include "gpio.h"
#include <stm32f4xx.h>

GPIO_TypeDef * Ports[9] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI };

// Инициализация
void gpio_Init(void)
{
    // Тактирование во все порты!
    RCC->AHB1ENR |= 0 |
                    RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN |
                    RCC_AHB1ENR_GPIOFEN |
                    RCC_AHB1ENR_GPIOGEN |
                    RCC_AHB1ENR_GPIOHEN |
                    RCC_AHB1ENR_GPIOIEN |
                    0;
}

// Настроить вывод на выход
void gpio_DigitalOutput(uint8_t Port, uint8_t Pin)
{
    // Очистим поле
    Ports[Port]->MODER &= ~(0x03UL << (Pin * 2));
    
    // Режим выхода
    // GP Output = 1
    Ports[Port]->MODER |=  (0x01UL << (Pin * 2));
}

// Настроить вывод на вход
void gpio_DigitalInput(uint8_t Port, uint8_t Pin)
{
    // Очистим поле
    // GP Input = 0
    Ports[Port]->MODER &= ~(0x03UL << (Pin * 2));
}

// Высокий уровень на ножке
void gpio_HighLevel(uint8_t Port, uint8_t Pin)
{
    Ports[Port]->BSRRL = (1UL << (Pin & 0x0F));
}

// Низкий уровень на ножке
void gpio_LowLevel(uint8_t Port, uint8_t Pin)
{
    Ports[Port]->BSRRH = (1UL << (Pin & 0x0F));
}

// Значение на ножке
bool gpio_Value(uint8_t Port, uint8_t Pin)
{
    return ((Ports[Port]->IDR & (1 << Pin)) != 0);
}


// Включить Open Drain (иначе двухтактный)
void gpio_OpenDrain(TPort Port, uint8_t Pin, bool Enable)
{
    if(Enable)
    {
        Ports[Port]->OTYPER |= (1 << Pin);
    }
    else
    {
        Ports[Port]->OTYPER &= ~(1 << Pin);
    }
}

// Установить для вывода альтернативную функцию (0-3)
void gpio_SetAlternateFunction(uint8_t Port, uint8_t Pin, uint8_t Function)
{
    Function &= 0x0F;
    
    // AF = 2
    Ports[Port]->MODER &= ~(0x03UL << (Pin * 2));
    Ports[Port]->MODER |=  (0x02UL << (Pin * 2));
    
    if(Pin < 8)
    {
        Pin &= 0x07;
        
        // Очистим, потом запишем
        Ports[Port]->AFR[0] &= ~(0x0F << (Pin * 4));
        Ports[Port]->AFR[0] |= ((uint32_t)Function << (Pin * 4));
    }
    else
    {
        Pin &= 0x07;
        
        // Очистим, потом запишем
        Ports[Port]->AFR[1] &= ~(0x0F << (Pin * 4));
        Ports[Port]->AFR[1] |= ((uint32_t)Function << (Pin * 4));
    }
}
