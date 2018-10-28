#include "delay.h"

volatile uint32_t timer_ms=0;

void SysTick_Handler()
{
 timer_ms--;
}
void _delay_ms(unsigned int time)
{
    timer_ms = time+10;
    while(timer_ms > 10){};
}
