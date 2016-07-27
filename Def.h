
//******ESC set**********//
#define _PC_PWM_
  //#define _PC_PWM_8ms_
  #define _PC_PWM_2ms_
//#define _FAST_PWM_

#define ESC_CALIB_CANNOT_FLY  // ESC calibrating

//*****sensor read mode****//
#define _BYPASS_MODE_
//#define _MASTER_MODE_

#define FAILSAFE

//#define GYRO_SMOOTHING {45, 45, 50} // ??
#define INFLIGHT_ACC_CALIBRATION
//#define MMGYRO 10 //// Moving Average Gyros by Magnetron1
#define _abs(X)                   ( (X < 0) ? -(X) : X ) // wrong: ( (X < 0) ? -X : X )
#define CONSTRAIN(AMT, LOW, HIGH) (AMT < LOW) ? LOW : ( (AMT > HIGH)?HIGH:AMT )
#define _min(X, Y)                ( (X < Y) ? X : Y ) 
#define _max(X, Y)                ( (X > Y) ? X : Y )

#define BIT(X)      (1<<(X))
#define BITSET(X,Y) ((X)|=BIT(Y))
#define BITCLR(X,Y) ((X)&=~BIT(Y))
#define BITTOG(X,Y) ((X)~=BIT(Y))
#define BITTST(X,Y) ((X)&BIT(Y))

#define LEDPIN_OUTPUT DDRB |= 0X20
#define LED_ON        PORTB|=0X20
#define LED_OFF       PORTB&=~0X20
#define LED_TOGGLE    PINB |= 1<<5

#define MAXTHROTTLE 2000 //1850 or 2000
#define MINTHROTTLE 1100 //1150
#define FAILSAFE_THROTTLE (MINTHROTTLE + 200)
#define MINCHECK 1100
#define MAXCHECK 1900
#define MINCOMMAND 1000

//#define PID_CONTROLLER 1
#define PID_CONTROLLER 2