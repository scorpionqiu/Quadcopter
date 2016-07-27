#include"MAIN.h"

//static unsigned long countMicroseconds = 0;
volatile unsigned long rawTime = 0;
void initTimer2(void)
{
//  SREG|=0X80; //开中断总
                   //TCCR2B|=0X05; //0x00000101  分频clk/128  解析度8us * 256 =2048 us
  TCCR2B |= 0x04; // 0x0000 0100   clk(16MHz)/64   resolution : 4us  * 256 = 1024 us
  TIMSK2|=0X01;   // overflow interrupt enable
} 

#pragma vector = TIMER2_OVF_vect
__interrupt void IsrTimer2(void)
{
  rawTime += 1;
}
/* a resolution of 4 microseconds,  approximately 70 minutes */
unsigned long micros(void)  
{
  unsigned long tempTime = (unsigned long)TCNT2;
  unsigned long tempRawTime = rawTime;
  return (tempRawTime<<10) + (tempTime<<2);
                              // 移位操作优先级较四则运算低，需加括号
}
/*
void DelayMs(unsigned int ms)
{
  unsigned long delayUs = micros() + (unsigned long)ms*1000;
  while( micros()<delayUs );
}*/

//#pragma vector=TIMER2_OVF_vect
//__interrupt void Timer2_Service(void)
//{
//  static unsigned char cnt;
//  if(!cnt--)
//  {
//    cnt=7;
//    CountMilliseconds+=1;  //2ms +1 
//  }
//}
//
//unsigned int SetDelay(unsigned int w)
//{
//  return(CountMilliseconds+w-1);
//}
//
//char CheckDelay(unsigned int k)
//{
//  return(((k-CountMilliseconds)&0x8000)>>8);
//}
//void Delay_ms(unsigned int ms)  //ms延时
//{
//  unsigned int t;
//  t=SetDelay(ms);
//  while(!CheckDelay(t));
//}
//
//void Delay_us(unsigned char us)  //0-127us
//{
//  unsigned char Us=(unsigned char)TCNT2+(us+us);
//  while((TCNT2-Us)&0x80);
//}