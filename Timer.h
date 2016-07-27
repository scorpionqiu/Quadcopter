#ifndef _TIMER_H_
#define _TIMER_H_
#include "intrinsics.h"
#define xtal 16   //16M ¾§Õñ
#define delay_us(x) __delay_cycles((unsigned long)(x*xtal));
#define delay_ms(x) __delay_cycles((unsigned long)x*xtal*1000);
#define delay_s(x)  __delay_cycles((unsigned long)(x*xtal*1000000));   
//extern volatile unsigned int CountMilliseconds;

void initTimer2(void);
unsigned long micros(void);
//void DelayMs(unsigned int ms);
//void Delay_ms(unsigned int);
//unsigned int SetDelay(unsigned int w);
//char CheckDelay(unsigned int k);

#endif