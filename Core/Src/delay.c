#include "delay.h"

void Delay_us(uint16_t time)
{
    uint16_t i;

    for (i = 0; i < time; i++)
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
		__NOP();
        __NOP();
		__NOP();
        __NOP();
        __NOP();
		__NOP();
        __NOP();
		__NOP();
    }
}

void Delay_ms(uint16_t time)
{
    uint16_t i;

    for (i = 0; i < time * 1000; i++)
    {
        Delay_us(1);
    }
}