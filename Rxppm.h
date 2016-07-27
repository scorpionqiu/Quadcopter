#ifndef _RXPPM_H_
#define _RXPPM_H_
#include "stdbool.h"

#define RC_CHANS  6  // The number of channels
      //#define Roll     0  //ch1  ²Î¼û Types.h
      //#define Pitch    1  //ch2
      //#define Throttle 2  //ch3
      //#define Yaw      3  //ch4
//#define VERIFY(T) (123<T && T<249)
#define VERIFY( T ) (1000<T && T< 2000) // from 1000us to 2000us
#define ROLLPIN     2 //PD2
#define PITCHPIN    3 //PD3
#define YAWPIN      4 //PD4
#define THROTTLEPIN 7 //PD7
#define FAILSAFE_DETECT_THRESHOLD 980
void initPPM(void);
void ComputeRC(void);

#endif
//#define FAILSAFE