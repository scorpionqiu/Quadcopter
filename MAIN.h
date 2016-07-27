#ifndef _MAIN_H_
#define _MAIN_H_
//#include <intrinsics.h>
#include <iom328p.h>
#include <math.h>
#include "Def.h"
#include "Types.h"
#include "Output.h"
#include "Timer.h"
#include "Usart.h"
#include "Twislave.h"
#include "Rxppm.h"
#include "Sensors.h"
#include "IMU.h"
#include "EEprom.h"

extern imu_t imu;
extern flags_t flags;
extern att_t att;
extern conf_t conf;
extern global_conf_t global_conf;
extern int gyroZero[3];
extern unsigned long currentTime; // recording current time,  used to MainLoop
extern unsigned int rcData[6];
extern int rcCommand[4];
extern int motor[4];
extern int axisPID[3];
extern int lookupPitchRollRC[5];
extern int lookupThrottleRC[11];
extern int magHold, headFreeModeHold;

extern int temp;

extern int annex650_overrun_count;
extern unsigned int calibratingA;
extern unsigned int calibratingG;
extern unsigned int calibratingB;
//extern unsigned char calibratingESC;
#if defined(INFLIGHT_ACC_CALIBRATION)
  extern unsigned int inflightCalibratingA;
  extern unsigned int accInflightCalibrationActive;
  extern unsigned int accInflightCalibrationMeasurementDone;
  extern unsigned int accInflightCalibrationSavetoEEProm;
//  extern unsigned int accInflightCalibrationArmed;
#endif
#ifdef FAILSAFE
  extern int failsafeEvents;
  extern volatile int failsafeCnt;
#endif

void ledTwinkle(unsigned int);
void ledSOS(unsigned char);
void AnnexCode(void);
#endif