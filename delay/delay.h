#ifndef DELAY_H_INCLUDED
#define DELAY_H_INCLUDED

#include <stdint.h>

/*remember to write this somewhere before first use of _delay_ms:
    SysTick_Config(SystemCoreClock / 1000);
    */
void SysTick_Handler();
void _delay_ms(unsigned int);

#endif /* DELAY_H_INCLUDED */
