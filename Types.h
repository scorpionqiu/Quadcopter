#ifndef TYPES_H_
#define TYPES_H_
enum rc {
  ROLL,  //0 ·­¹ö
  PITCH, //1 ¸©Ñö
  YAW,   //2 Æ«º½
  THROTTLE,//3 ÓÍÃÅ
  AUX1,
  AUX2
};

enum pid {
  PIDROLL,  //0
  PIDPITCH, //1
  PIDYAW,   //2
  PIDALT,   //3
  PIDPOS,   //4
  PIDPOSR,  //5
  PIDNAVR,  //6
  PIDLEVEL, //7
  PIDMAG,   //8
  PIDVEL,   //9
  PIDITEMS // 10
};

enum box {
  BOXARM,
  BOXANGLE,
  BOXHORIZON
};

struct pid_ {
  unsigned char P8;
  unsigned char I8;
  unsigned char D8;
};

struct servo_conf_ {
  int min;
  int max;
  int middle;
  char rate;
};

typedef struct {
  struct pid_ pid[PIDITEMS]; // 10 * 3 = 30 B
  unsigned char rcRate8;
  unsigned char rcExpo8;
  unsigned char rollPitchRate;
  unsigned char yawRate;
  unsigned char dynThrPID;
  unsigned char thrMid8;
  unsigned char thrExpo8;
  int mag_declination; // 2 B
  int angleTrim[2];   // 4 B
#if defined(GYRO_SMOOTHING)
  unsigned char Smoothing[3]; // 3 B
#endif
#if defined(FAILSAFE)
  int failsafe_throttle;  // 2 B
#endif
  int minthrottle;        // 2 B
  unsigned char checksum; // 1 B
} conf_t;

typedef struct {
  int accADC[3];
  int gyroData[3];
  int magADC[3];
  int gyroADC[3];
  int accSmooth[3]; // the result of LPF 
} imu_t;

typedef struct {
  unsigned char ARMED; 
  unsigned char OK_TO_ARM;
  unsigned char SMALL_ANGLE_25 : 1;
  unsigned char CALIBRATE_MAG : 1;
  unsigned char ANGLE_MODE;
  unsigned char HORIZON_MODE;
} flags_t;

typedef struct {
  int angle[2];
  int heading;
 } att_t;

typedef struct {
  unsigned char currentSet; //1 bytes
  int accZero[3]; //6 bytes
  int magZero[3]; //6 bytes
  unsigned int flashsum;// 2 bytes
  unsigned char checksum; //1 bytes
} global_conf_t;


#endif