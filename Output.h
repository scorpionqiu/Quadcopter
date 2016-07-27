
#ifndef _OUTPUT_H_
#define _OUTPUT_H_

//extern unsigned char PWM0A;  //D5
//extern unsigned char PWM0B;  //D6
//extern unsigned char PWM1A;  //D9
//extern unsigned char PWM1B;  //D10

#define PWM_A0_ON {TCCR0A|=0X81;TCCR0B=0X04;DDRD|=0X40;}
#define PWM_B0_ON {TCCR0A|=0X21;TCCR0B=0X04;DDRD|=0X20;}
#define PWM_A1_ON {TCCR1A|=0X81;TCCR1B=0X04;DDRB|=0X02;}
#define PWM_B1_ON {TCCR1A|=0X21;TCCR1B=0X04;DDRB|=0X04;}
#define PWM_ALL_ON {DDRD|=0X40;DDRD|=0X20;DDRB|=0X02;DDRB|=0X04;}

#define PWM_A0_OFF {TCCR0A&=~0XC0;PORTD&=~0X40;}
#define PWM_B0_OFF {TCCR0A&=~0X30;PORTD&=~0X20;}
#define PWM_A1_OFF {TCCR1A&=~0XC0;PORTB&=~0X02;}
#define PWM_B1_OFF {TCCR1B&=~0X30;PORTB&=~0X04;}
#define PWM_ALL_OFF {TCCR0A&=~0XF0;TCCR1A&=~0XF0;PORTD&=~0X60;PORTB&=~0X06;}

#ifdef _PC_PWM_
  #ifdef _PC_PWM_8ms_
    #define ThrotTop    78  //upper limit, influenced by a new ESC
    #define ThrotBottom 19  //lower limit
    #define ThrotInit   25
  #endif
  #ifdef _PC_PWM_2ms_
    #define ThrotTop    250 // can be try Upper
    #define ThrotBottom 125 // can be try Lower
    #define ThrotInit   128
  #endif
#endif
#ifdef _FAST_PWM_
  #define ThrotTop    128  // can be try Upper
  #define ThrotBottom 64   // can be try Lower
  #define ThrotInit   70
#endif

void PWM_Int(void);
void SetPWM(void);

void TestInput(char, char, char, char);

void mixTable(void);
void writeMotors(void);
void initOutput(void);
void Activate(void);
#endif
