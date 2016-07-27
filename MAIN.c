#include "MAIN.h"
// ******************
// rc functions
// ******************
#define ROL_LO  (1<<(2*ROLL))//0x0000 0001
#define ROL_CE  (3<<(2*ROLL))//0x0000 0011
#define ROL_HI  (2<<(2*ROLL))//0x0000 0010
#define PIT_LO  (1<<(2*PITCH))//0x0000 0100
#define PIT_CE  (3<<(2*PITCH))//0x0000 1100
#define PIT_HI  (2<<(2*PITCH))//0x0000 1000
#define YAW_LO  (1<<(2*YAW))//0x0001 0000
#define YAW_CE  (3<<(2*YAW))//0x0011 0000
#define YAW_HI  (2<<(2*YAW))//0x0010 0000
#define THR_LO  (1<<(2*THROTTLE))//0x0100 0000
#define THR_CE  (3<<(2*THROTTLE))//0x1100 0000
#define THR_HI  (2<<(2*THROTTLE))//0x1000 0000

imu_t imu;
flags_t flags;
att_t att;
conf_t conf; //成员变量 需初始化
global_conf_t global_conf;

int temp;

int gyroZero[3] = {0, 0, 0};
int rcCommand[4];
int motor[4];
int axisPID[3];
static int dynP8[2], dynD8[2];
int lookupPitchRollRC[5];
int lookupThrottleRC[11];
int magHold, headFreeModeHold;

int annex650_overrun_count = 0;
unsigned int calibratingA = 0;
unsigned int calibratingG;
unsigned int calibratingB = 0;
#if defined(INFLIGHT_ACC_CALIBRATION)
  unsigned int inflightCalibratingA = 0;
  unsigned int accInflightCalibrationActive = 0;
  unsigned int accInflightCalibrationMeasurementDone = 0;
  unsigned int accInflightCalibrationSavetoEEProm = 0;
#endif
unsigned long currentTime = 0; // used to MainLoop for achieve current time in microsecond
unsigned long previousTime = 0;
unsigned int  cycleTime = 0; //to achieve a full loop, it can differ a little and is taken into account in the PID loop
#ifdef FAILSAFE
  int failsafeEvents = 0;
  volatile int failsafeCnt = 0;
#endif
  
void ledTwinkle(unsigned int repeat){
//  unsigned char on = 0;
  for(unsigned int i = 0; i < repeat; i++){
//    if(on == 0xff) 
//      LED_ON;
//    if(on == 0)
//      LED_OFF;
    LED_TOGGLE;
    delay_ms(60);
//    on = ~on;
  }
  LED_OFF;
}
void ledSOS(unsigned char repeat){
  for(unsigned char i = 0; i < repeat; i++){
    LED_ON;delay_ms(180); LED_OFF;delay_ms(180);
    LED_ON;delay_ms(180); LED_OFF;delay_ms(180);
    LED_ON;delay_ms(180); LED_OFF;delay_ms(180);
    LED_ON;delay_ms(60);  LED_OFF;delay_ms(60);
    LED_ON;delay_ms(60);  LED_OFF;delay_ms(60);
    LED_ON;delay_ms(60);  LED_OFF;delay_ms(60);
  }
}

void AnnexCode(void){ // <116us current
  
  /* code about rcData[4] processing */
  int tmp, tmp2;
  unsigned char axis, prop1, prop2;
  prop2 = 128;
  
  /* do something depend on THROTTLE */
  
  for(axis = 0; axis < 3; axis++){//ROLL, PITCH, YAW
    tmp = _min( _abs((int)rcData[axis] - 1500), 500 );// define MIDRC 1500
    
    if(tmp < 15) tmp = 0; // zero point range( Dead band: 15 ) 
    else     tmp -= 15;
    if(axis!=2){
      tmp2 = tmp>>7;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ( (tmp-(tmp2<<7)) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2])>>7 );
      prop1 = 128-((unsigned int)conf.rollPitchRate*tmp >>9); //conf.rollPitchRate = 0 (from LoadDefaults()) ; // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = (unsigned int)prop1*prop2 >>7; // prop1: max is 128   prop2: max is 128   result prop1: max is 128
      dynP8[axis] = (unsigned int)conf.pid[axis].P8*prop1 >>7; // was /100, is /128 now
      dynD8[axis] = (unsigned int)conf.pid[axis].D8*prop1 >>7; // was /100, is /128 now   
    }else{
      rcCommand[axis] = tmp;
    }
    if(rcData[axis]<1500) 
      rcCommand[axis] = -rcCommand[axis];
  }
  
  /* process rcData[THROTTLE] */
  
  tmp = CONSTRAIN(rcData[THROTTLE], 1000, 2000);
  rcCommand[THROTTLE] = tmp;
  
  SerialCom(); //Send to GUI; Receive from GUI waits to implement
}

void Setup(){
  LEDPIN_OUTPUT;
  initUSART( 115200 ); //baud 9600 14400 19200 28800 38400 57600 76800 115200
  initOutput();
  readGlobalSet();
  if( !readEEPROM() ){ // false means conf changed, so read again.
    LED_ON;
  }
  initTimer2();
  initI2C();
  initPPM();     //初始化遥控器接收  
  initSensors();
  SREG |= BIT(7); //开中断
  calibratingG = 512; // gyro calibration when booting
}

void go_disarm(void){
  if(flags.ARMED) {
    LED_OFF;
    flags.ARMED = 0;
  }
}
void go_arm(void){
  if(calibratingG == 0 && failsafeCnt < 2){
    if( !flags.ARMED ){
      flags.ARMED = 1;
      headFreeModeHold = att.heading; // current heading (before flying)
      magHold = att.heading; // hold on current heading before flying
      LED_ON;
    } else go_disarm();
  }//else
}

void main(void)
{
  Setup();
  
  flags.HORIZON_MODE = 1;
  flags.ANGLE_MODE = 0;
   
  while(1)
  {
    static unsigned char rcDelayCommand;
    static unsigned char rcSticks;
    unsigned char axis, i;
    int errorAngle;
    int delta;
    int PTerm = 0, ITerm = 0, DTerm;
#if PID_CONTROLLER == 1
    int error;
    int rc;
    int PTermACC, ITermACC;
    static long errorGyroI_YAW;
    static int delta1[2], delta2[2];
    static int errorGyroI[2];
    static int lastGyro[2] = {0, 0};
    static int errorAngleI[2] = {0, 0};
#elif PID_CONTROLLER == 2
    static int delta1[3], delta2[3];
    static long errorGyroI[3] = {0, 0, 0};
    static int lastError[3] = {0, 0, 0};
    int deltaSum;
    int AngleRateTmp, RateError;
#endif
    static unsigned long rcTime = 0;
    static int initialThrottleHold;
    static unsigned long timestamp_fixated = 0;
    long prop = 0;
    
    if(currentTime > rcTime){
      rcTime = currentTime + 20000;// + 20ms
      ComputeRC();
      #if defined(FAILSAFE) //遥控信号失联
      if( failsafeCnt > 50 && flags.ARMED ){
        for( i = 0; i < 3; i++ ) rcData[i] = 1500; //hold on in level
        rcData[THROTTLE] = conf.failsafe_throttle; //throttle hole on; config in eeprom
        if( failsafeCnt > 5*210 ) {
          /*  */
        }
      }
      if(failsafeCnt>50 && !flags.ARMED) {
        go_disarm();
        flags.OK_TO_ARM = 0;
      }
      failsafeCnt++;
      #endif
      
      unsigned char stTmp = 0;
      for( i = 0; i < 4; i++){
        stTmp >>= 2;
        if(rcData[i] > MINCHECK) stTmp |= 0x80;
        if(rcData[i] < MAXCHECK) stTmp |= 0x40;
      }
      if(stTmp == rcSticks){
        if(rcDelayCommand<250) rcDelayCommand++;
      }else rcDelayCommand = 0;
      rcSticks = stTmp;
      
      if(rcData[THROTTLE] <= MINCHECK){ // <=1100  
      #if PID_CONTROLLER == 1
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0;
        errorGyroI_YAW = 0;
      #elif PID_CONTROLLER == 2
        errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0;
        errorGyroI[YAW] = 0;
      #endif
      }
      if(rcDelayCommand == 20){ // set from rc actions when disarm
        if(flags.ARMED) {
          if( rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE ) go_disarm(); //disarm via yawRC with far left
        }else{
          if( rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE &&
             flags.CALIBRATE_MAG == 0  && calibratingA == 0 ){
               go_arm();  // arm via yawRC with far right
             }
            
          if( rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
            calibratingG = 512; // GYRO calibration
            /* baro calibration */
            /* GPS_HOME_POSITION */
          }
          #if defined(INFLIGHT_ACC_CALIBRATION)
          else if(rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI){
           /*  do something  */
          }            
          #endif
          else if(rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) calibratingA = 512; // ACC calibration
          else if(rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) flags.CALIBRATE_MAG = 1;// MAG calibration
          else if(rcSticks == THR_LO + YAW_CE + PIT_LO + ROL_LO){
            __EEPUT(0x3FF, 10); // Reload power to calibrate ESC
            while(1) ledSOS(10);
          }
        }
      }
      #if defined(INFLIGHT_ACC_CALIBRATION)
        /*  do something  */
      #endif     
      
    }else{
      unsigned char taskOrder = 0;
      switch(taskOrder){
      case 0:
        taskOrder++;
        if( Mag_getADC() ) break;
      case 1://GPS
        taskOrder++;
        break;
      case 2://Baro
        taskOrder++;
        break;
      case 3://Sonar
        taskOrder++;
        break;
      default:
        taskOrder = 0;break;
      }
    }
    
    ComputeIMU();
    currentTime = micros();
    cycleTime = currentTime - previousTime; // For intergrated T cycle
    previousTime = currentTime;
    
    if(_abs(rcCommand[YAW]) < 70){// Hold on heading with dead band
      int dif = att.heading - magHold;
      if(dif <= -180) dif += 360;
      if(dif >= +180) dif -= 360;
      if(flags.SMALL_ANGLE_25)
        rcCommand[YAW] -= dif*conf.pid[PIDMAG].P8>>5;// Self-hold control  default: 40/32
    }else magHold = att.heading; // Hold on after changing heading
    
#if PID_CONTROLLER == 1
    if ( flags.HORIZON_MODE ) prop = _min( _max( _abs(rcCommand[PITCH]), _abs(rcCommand[ROLL]) ), 512 );
    
    for(axis = 0; axis < 2; axis++){ //ROLL, PITCH
      rc = rcCommand[axis]<<1; // rc: [-1000; 1000]
      error = rc - imu.gyroData[axis]; // <1000 - <8192
      errorGyroI[axis] = CONSTRAIN( errorGyroI[axis]+error, -16000, 16000 );
      if( _abs(imu.gyroData[axis]) > 640 ) errorGyroI[axis] = 0;
      ITerm = (errorGyroI[axis]>>7) * conf.pid[axis].I8 >>6;
      PTerm = (long)rc * conf.pid[axis].P8 >>6;
      if(flags.ANGLE_MODE || flags.HORIZON_MODE){
        errorAngle    = CONSTRAIN(rc , -500, 500) - att.angle[axis];// + conf.angleTrim[axis];  +/-(500/10)degree limited
        errorAngleI[axis] = CONSTRAIN( errorAngleI[axis]+errorAngle, -10000, 10000);
        
        PTermACC = (long)errorAngle * conf.pid[PIDLEVEL].P8>>7;
        PTermACC = CONSTRAIN(PTermACC, -500, 500);
        ITermACC = ( (long)errorAngleI[axis] * conf.pid[PIDLEVEL].I8 )>>12;
        
        ITerm = ITermACC + ( (ITerm-ITermACC) * prop >>9 );
        PTerm = PTermACC + ( (PTerm-PTermACC) * prop >>9 );
      }
      PTerm -= (long)imu.gyroData[axis] * dynP8[axis]>>6;
      
      delta          = imu.gyroData[axis] - lastGyro[axis];
      lastGyro[axis] = imu.gyroData[axis];
      DTerm          = delta1[axis] + delta2[axis] + delta;
      delta2[axis]   = delta1[axis];
      delta1[axis]   = delta;
      
      axisPID[axis] = PTerm + ITerm - DTerm;
    }
    
    /*  YAW pid control  */
    axisPID[YAW] = 0;
    
#elif PID_CONTROLLER == 2
    #define GYRO_I_MAX 256
    #define ACC_I_MAX 256
    prop = _min( _max(_abs(rcCommand[ROLL]),_abs(rcCommand[PITCH])), 500);
    for(axis = 0; axis < 3; axis++){
      if((flags.ANGLE_MODE||flags.HORIZON_MODE) && axis<2){
        // MODE relying on ACC, calculate error and limit the angle to 50 degrees max inclination
        errorAngle = CONSTRAIN((rcCommand[axis]<<1)/*+GPA_angle[axis]*/, -500, +500) - att.angle[axis] /*+ conf.angleTrim[axis]*/; 
      }
      if(axis == 2){// For YAW, always gyro-controlled(MAG corrention is applied to rcCommand)
        AngleRateTmp = ( (long)(conf.yawRate+27) * rcCommand[YAW] ) >>5;// /32
      }else{
        if(!flags.ANGLE_MODE){//control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
          AngleRateTmp = ( (long)(conf.rollPitchRate+20/*27*/) * rcCommand[axis] ) >>4;// /16 ??adjust rcCommand?
          if(flags.HORIZON_MODE){//mix up angle error to desired AngleRateTmp to add a little auto-level feel
            AngleRateTmp += ( (long)errorAngle * 128/*conf.pid[PIDLEVEL].I8*/ ) >>7/*8*/;// default: *32/256  ??it not make sense?
          }
        }else{//flags.ANGLE_MODE, control is angle based, so control loop is needed
          AngleRateTmp = ( (long)errorAngle * 25/*conf.pid[PIDLEVEL].P8*/ ) >>4;// *30/16  Only P-output in ANGLE_MODE,   repond slowly with low value 
        }
      }
      //------low-level gyro-based PID-------
      
      //Debug set:
//      AngleRateTmp = CONSTRAIN((rcCommand[axis]<<1)/*+GPA_angle[axis]*/, -500, +500);
//      if(axis == 1)temp = AngleRateTmp;
      
      //--PTerm---
      RateError = AngleRateTmp - imu.gyroData[axis];
      PTerm     = ( (long)RateError * 64/*conf.pid[axis].P8*/ ) >>7;// default: 28/128 -> 128/128 now
      //--ITerm---
      errorGyroI[axis] += (( (long)RateError * cycleTime ) >>11) * /*conf.pid[axis].I8*/10; //default:10  cycleTime/us[4544; 4844] /2048 = RateError *2 *10
      errorGyroI[axis] = CONSTRAIN(errorGyroI[axis], (long)-GYRO_I_MAX<<13, (long)GYRO_I_MAX<<13);// ?256<<13=2^8*2^13=2^21=
      ITerm            = errorGyroI[axis] >>13; // Limit to [-256; +256]
      //--DTerm---
      delta           = RateError - lastError[axis];
      lastError[axis] = RateError;
      delta        = ( (long)delta * ((unsigned int)0xFFFF/(cycleTime>>4)) ) >>6;//delta*(65535/~1000)/64
      deltaSum     = delta1[axis] + delta2[axis] + delta;
      delta2[axis] = delta1[axis];
      delta1[axis] = delta;
      DTerm        = (deltaSum * /*conf.pid[axis].D8*/7) >>8;//default: 7/256 -> 128/256
      
      // debug set
      //ITerm = 0; 
      //DTerm = 0;
      //Total PID Output
      axisPID[axis] = PTerm + ITerm + DTerm;
    }
#endif
    
    mixTable();
 //   if( flags.ARMED && calibratingG == 0 && calibratingA == 0)
    writeMotors();
    
    
//    delay_ms(100);
//    LED_TOGGLE;
//    Send_intData(rcCommand[ROLL]);USART_Transmit(';');
//    Send_intData(rcCommand[PITCH]);USART_Transmit(';');
//    Send_intData(rcCommand[YAW]);USART_Transmit(';');
//    Send_intData(rcCommand[THROTTLE]);USART_Transmit(';');
//    USART_Transmit('\n');

//    unsigned char c = 0;
//    c = rcData[THROTTLE]>>3;  
//    TestInput(c, c, c, c);

 
//    delay_ms(4); // the time same to CompueteIMU(), slower a little
//    USART_Transmit('0');
  }
}