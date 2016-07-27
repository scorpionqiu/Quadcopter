#include "MAIN.h"

void PWM_Int(void)
{
#ifdef _PC_PWM_
  TCCR0A=0XA1;       //Non-inverting mode,Phase Correct PWM mode
  TCCR1A=0XA1;       //Non-inverting mode,Phase Correct PWM mode
  #ifdef _PC_PWM_8ms_
    TCCR0B=0X04;       // 16M/256/510=122.549Hz   [8ms]
    TCCR1B=0X04;       //16M/256/(2*TOP)=122.549Hz;TOP=0XFF
  #endif
  #ifdef _PC_PWM_2ms_
    TCCR0B=0x03;//490.196hz 2.04ms
    TCCR1B=0x03;
  #endif
#endif
//**********************************//
#ifdef _FAST_PWM_
  TCCR0A=BIT(7)|BIT(5)|BIT(1)|BIT(0); //Fast PWM 8 bits, Non-inverting mode
  TCCR1A=BIT(7)|BIT(5)|BIT(1);  //Fast PWM 8 bits, Non-inverting mode
  TCCR0B=BIT(2);          // 16M/256/256=244.14Hz [4ms]
  TCCR1B=BIT(3)|BIT(2);   // 16M/256/256=244.14Hz [4ms]
#endif
}

#define THROT_H {motor[0]=ThrotTop;motor[1]=ThrotTop;motor[2]=ThrotTop;motor[3]=ThrotTop;}
   //#define THROT_H {PWM0A=250;PWM0B=250;PWM1A=250;PWM1B=250;}
#define THROT_L {motor[0]=ThrotBottom;motor[1]=ThrotBottom;motor[2]=ThrotBottom;motor[3]=ThrotBottom;}
   //#define THROT_L {PWM0A=125;PWM0B=125;PWM1A=125;PWM1B=125;}
#define RUN_INIT {motor[0]=ThrotBottom;motor[1]=ThrotBottom;motor[2]=ThrotBottom;motor[3]=ThrotBottom;}
//unsigned char PWM0A=0;  //D5
//unsigned char PWM0B=0;  //D6
//unsigned char PWM1A=0;  //D9
//unsigned char PWM1B=0;  //D10
void SetPWM(void)
{
  OCR0A = motor[0];
  OCR0B = motor[1];
  OCR1B = motor[2];
  OCR1A = motor[3];
}
void TestInput(char k1,char k2,char k3,char k4)
{
  OCR0A = k1;
  OCR0B = k2;
  OCR1A = k3;
  OCR1B = k4;
}

void Activate(void){
    
    PWM_ALL_ON;
    delay_ms(500);
      
    THROT_H;
    SetPWM();  
    ledTwinkle(10);
    delay_ms(4000);
    
    THROT_L;
    SetPWM();
    ledTwinkle(10);
    while(1){
     ledSOS(10);
     delay_ms(3000);   
    }

//    LED_ON;
//    RUN_INIT;
//    SetPWM();
//    LED_ON;
//       PWM_ALL_OFF;
//       delay_ms(2000);
//       PWM_ALL_ON;
}

void initOutput(void){
  PWM_Int();
#if defined(ESC_CALIB_CANNOT_FLY)
    unsigned char calibratingESC = 0;
    __EEGET(calibratingESC, 0x3FF);
    if(calibratingESC == 10){
      __EEPUT(0x3FF, 0xFF);
      Activate();   //Set throttle range
    }    
#endif
  LED_OFF; 
  RUN_INIT; // Lowest throttle
  SetPWM();
  PWM_ALL_ON;
  LED_ON;
  delay_ms(5000); // can be quick
  LED_OFF;
}

void mixTable(void){
  //mode: X
  int maxMotor;
  unsigned char i;
  #define PIDMIX(X, Y, Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + axisPID[YAW]*Z
  motor[0] = PIDMIX(+1,-1,-1); //front_left
  motor[1] = PIDMIX(-1,-1,+1); //front_right
  motor[2] = PIDMIX(-1,+1,-1); //rear_right
  motor[3] = PIDMIX(+1,+1,+1); //reat_left
//  motor[0] = PIDMIX(+1, -1, 0);//test pitch
//  motor[1] = PIDMIX(-1, -1, 0);
//  motor[2] = PIDMIX(-1, +1, 0);
//  motor[3] = PIDMIX(+1, +1, 0);
  
  maxMotor = motor[0];
  for(i = 1; i < 4; i++)
    if(motor[i] > maxMotor) maxMotor = motor[i];
  for(i = 0; i < 4; i++){
    if(maxMotor > MAXTHROTTLE)
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = CONSTRAIN( motor[i], conf.minthrottle, MAXTHROTTLE );
    if( rcData[THROTTLE] < MINCHECK )
      motor[i] = conf.minthrottle;
    if(!flags.ARMED)
      motor[i] = MINCOMMAND;
  }
}

void writeMotors(void){
  OCR0A = motor[0] >> 3;
  OCR0B = motor[1] >> 3;
  OCR1B = motor[2] >> 3;
  OCR1A = motor[3] >> 3;
}