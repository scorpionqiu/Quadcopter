#include "MAIN.h"

//static unsigned long beginTime = 0;
volatile unsigned int rcChanValue[RC_CHANS]={0,0,0,0,0,0};
unsigned int  rcData[6] = {0,0,0,0,0,0};
static unsigned char rcPins[6] = {(1<<2)/*ROLL*/, (1<<3)/*PITCH*/, (1<<4)/*YAW*/, (1<<7)/*THROTTLE*/, 0, 0};

void initPPM(void)
{
//  EICRA = BIT(2) | BIT(0); //0x00000101  interrupt with any logical change
//  EIMSK = BIT(1) | BIT(0); //0x00000011  INT1 INT0 enable  (PD3,PD2)
//  PCICR = BIT(2) | BIT(0); //配置PINB和PIND为 Pin Change Interrupt
//  PCMSK2= BIT(7);          //PD7(PCINT23) enable  PD7(D7)
//  PCMSK0= BIT(0);          //PB0(PCINT0)  enable  PB0(D8)
  PCICR = BIT(2);
  PCMSK2 = BIT(2) | BIT(3) | BIT(4) | BIT(7);
}

#if defined(FAILSAFE)
  #define RX_PIN_CHECK( chan_pos, rc_pin ) \
    if( mask & rcPins[chan_pos] ){\
      if( !(pin & rcPins[chan_pos]) ){\
        dTime = cTime - edgeTime[chan_pos];\
          if( VERIFY(dTime) ){\
            rcChanValue[chan_pos] = dTime;\
            if( rc_pin==ROLLPIN || rc_pin==PITCHPIN || rc_pin==YAWPIN ||\
              rc_pin==THROTTLEPIN && dTime>FAILSAFE_DETECT_THRESHOLD )\
                goodPulses |= ( 1<<rc_pin );\
          }\
       }else edgeTime[chan_pos] = cTime;\
     } 
#else
  #define RX_PIN_CHECK( chan_pos, rc_pin ) \
    if( mask & rcPins[chan_pos] ){\
      if( !(pin & rcPins[chan_pos]) ){\
        dTime = cTime - edgeTime[chan_pos];\
        if( VERIFY(dTime) )\
          rcChanValue[chan_pos] = dTime;\
      } else edgeTime[chan_pos] = cTime;\
    }
#endif

#pragma vector = PCINT2_vect
__interrupt void IsrPCINT2(void)
{
  unsigned char mask;
  unsigned char pin;
  unsigned long cTime;
  unsigned int  dTime;
  static unsigned long edgeTime[6];
  static unsigned char lastPIND;
  #if defined(FAILSAFE)
    static unsigned goodPulses;
  #endif
  pin = PIND;
  mask = pin ^ lastPIND; //Check pin change
  cTime = micros();
  __enable_interrupt();
  lastPIND = pin;
  RX_PIN_CHECK( ROLL,     2 );
  RX_PIN_CHECK( PITCH,    3 );
  RX_PIN_CHECK( YAW,      4 );
  RX_PIN_CHECK( THROTTLE, 7 );
  
#if defined( FAILSAFE )
  if( goodPulses == (1<<ROLLPIN)+(1<<PITCHPIN)+(1<<YAWPIN)+(1<<THROTTLEPIN) ){
    goodPulses = 0;
    if( failsafeCnt > 20 ) failsafeCnt -= 20; else failsafeCnt = 0;
  }
#endif
}

////Roll  0  ch1(->D3)
//#pragma vector = INT1_vect    
//__interrupt void IsrInt1(void)
//{
//  unsigned long temp0;
//  __enable_interrupt();
//  if( PIND & BIT(3) ){
//    beginTime = micros();
//    }
//  else{
//    temp0 = micros() - beginTime;
//    if( VERIFY(temp0) )
//      rcChanValue[ROLL] = (int)temp0;
//  }
//}
////Pitch 1 ch2(->D7)
//#pragma vector = PCINT2_vect
//__interrupt void IsrPCINT2(void)
//{
//  unsigned long temp1;
//  __enable_interrupt();
//  if( PIND & BIT(7) ){
//    beginTime = micros();
//  }
//  else{
//    temp1 = micros() - beginTime;
//    if( VERIFY(temp1) )
//      rcChanValue[PITCH] = (int)temp1;
//  }
//}
////Yaw  2  ch4(->D8)
//#pragma vector = PCINT0_vect
//__interrupt void IsrPCINT0(void)
//{
//  unsigned long temp2;
////  static unsigned char yawTime;
//  __enable_interrupt();
//  if( PINB & BIT(0) ){
//    beginTime = micros();
//  }
//  else{
//    temp2 = micros() - beginTime;
//    if( VERIFY(temp2) )
//      rcChanValue[YAW] = (int)temp2;
//  }
//}
//
//// Throttle  3  ch3(->D2)
//#pragma vector = INT0_vect    
//__interrupt void IsrInt0(void)
//{
//  unsigned long temp3 = 0;
//  //static unsigned char throtTime; 
//  __enable_interrupt();
//  if( PIND & BIT(2) ){
//    beginTime = micros();
//  }
//  else{
//    temp3 = micros() - beginTime;
//    if( VERIFY(temp3) )
//      rcChanValue[THROTTLE] = (int)temp3;
//  }
//}

unsigned int ReadChannel(unsigned char chan)
{
  unsigned int data;
  unsigned char oldSREG;
  oldSREG = SREG; __disable_interrupt();
  data = rcChanValue[chan];
  SREG = oldSREG;
  return data;
}
void ComputeRC(void) //均值滤波
{
  static unsigned char rc4DataIndex = 0;
  static unsigned int rc4Data[RC_CHANS][4];
  static unsigned int rcDataMean[4];
  rc4DataIndex++; // update the eldest data at each channel
  if(rc4DataIndex == 4) rc4DataIndex = 0;
  for(unsigned char chan=0; chan<(RC_CHANS-2); chan++)  //4 channels be used 
  {
    rc4Data[chan][rc4DataIndex] = ReadChannel(chan);
    
    rcDataMean[chan] = 0;
    for(unsigned char i = 0; i < 4;i++) rcDataMean[chan] += rc4Data[chan][i];
    rcDataMean[chan] = ( (rcDataMean[chan]+2) >> 2); //4个数之和 再除以4
    if( rcDataMean[chan] < rcData[chan]-3 )
      rcData[chan] = rcDataMean[chan] +2;
    if( rcDataMean[chan] > rcData[chan]+3)
      rcData[chan] = rcDataMean[chan] -2;
  }
}
