#include "MAIN.h"
void getEstimatedAttitude(void);
void ComputeIMU(void) // 3520us(most time) -> 4100us  
{
  unsigned char axis;
  static int gyroADCprevious[3] = {0, 0, 0};
  int gyroADCp[3];
  int gyroADCinter[3];
  static unsigned long timeInterleave = 0;
  
#if defined(NUNCHUCK)
  
#else
  Acc_getADC();
  getEstimatedAttitude();
  Gyro_getADC();
  for(axis = 0; axis < 3; axis++){
    gyroADCp[axis] = imu.gyroADC[axis];
  }
  timeInterleave = micros();
  
  AnnexCode();

  unsigned char t = 0;
  while( (unsigned int)(micros()-timeInterleave)<650) t = 1;
  
  if(!t) annex650_overrun_count++; //表示发送数据时间超时
  Gyro_getADC(); //consecutive reading of Gyro. delay time <650us
  for(axis = 0; axis < 3; axis++){
    gyroADCinter[axis] = imu.gyroADC[axis] + gyroADCp[axis];
    imu.gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis])/3; // Something wrong is static unsigned int gyroADCprevious[3] defined, thanks god! i hate myself
    gyroADCprevious[axis] = gyroADCinter[axis]>>1;
    
    //imu.accADC[axis] = 0;
  }
//  #if defined(GYRO_SMOOTHING)
//  static int gyroSmooth[3] = {0, 0, 0};
//  for(axis = 0; axis < 3; axis++){
//    imu.gyroData[axis] = (int)( ( (long)( (long)gyroSmooth[axis]*(conf.Smoothing[axis]-1) ) + imu.gyroData[axis]+1 ) / conf.Smoothing[axis] );
//    gyroSmooth[axis] = imu.gyroData[axis];
//  }
//  #endif
  
#endif
}

typedef struct float_vector{
  float X, Y, Z;
} t_float_vector_def;

typedef union{
  float A[3];
  t_float_vector_def V;
} t_float_vector;

typedef struct long_vector{
  long X, Y, Z;
} t_long_vector_def;

typedef union{
  long A[3];
  t_long_vector_def V;
} t_long_vector;

float InvSqrt(float x){  //需要验证, Done
  union{
    long i;
    float f;
  } conv;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

int _atan2(long y, long x){
  float z = (float)y / x;
  int a;
  if( fabs(y) < fabs(x)){
    a =(int)( 573 * z / (1.0f + 0.28f * z *z) );
    if(x<0){
      if(y<0) a -= 1800;
      else a += 1800;
    }
  }else{
    a =(int)(900 - 573 *z / (z * z + 0.28f) );
    if(y<0) a -= 1800;
  } 
  return a;
}

void rotateV(struct float_vector *v, float* delta){
  t_float_vector_def v_tmp = *v;
  v->Z -= delta[ROLL] * v_tmp.X + delta[PITCH] * v_tmp.Y;  // v->X  =      1       -delta[yaw]    delta[roll]         v_tmp.X
  v->X += delta[ROLL] * v_tmp.Z - delta[YAW]   * v_tmp.Y;  // v->Y  =  delta[yaw]        1        delta[pitch]   *    v_tmp.Y
  v->Y += delta[PITCH]* v_tmp.Z + delta[YAW]   * v_tmp.X;  // v->Z  = -delta[roll] -delta[pitch]      1               v_tmp.Z
}

#define ACC_LPF_FACTOR   4     // a LPF of 16 for Accel
#define GYR_CMPFA_FACTOR 600 // the Gyro weight for Gyro/Acc complementary filter 
#define GYR_CMPFM_FACTOR 250 // the Gyro Weight for Gyro/Mag complementart filter
#define INV_GYR_CMPFA_FACTOR ( 1.0f / (GYR_CMPFA_FACTOR + 1.0f) )
#define INV_GYR_CMPFM_FACTOR ( 1.0f / (GYR_CMPFM_FACTOR + 1.0f) )

static long accLPF32[3]  = {0, 0, 1};
static float invG; // 1/G
// define Accelerometer Estimated
static t_float_vector EstA; 
static t_long_vector  EstA32;
// define Magnetometer Estimated
static t_float_vector EstM;
static t_long_vector  EstM32;
//static float Acc[3]

void getEstimatedAttitude(void)
{
  unsigned char axis;
  long accMag = 0;
  float scale, deltaGyroAngle[3]; // radian
  unsigned char validAcc;
  static unsigned long previousT;
  unsigned long currentT = micros(); //
  
  scale = (currentT - previousT) * GYRO_SCALE;
  previousT = currentT;
  for(axis = 0; axis < 3; axis++){
    deltaGyroAngle[axis] = imu.gyroADC[axis] * scale; // 临近时间内，旋转角度计算
    accLPF32[axis] -= accLPF32[axis]>>ACC_LPF_FACTOR; // 低通滤波
    accLPF32[axis] += imu.accADC[axis];
    imu.accSmooth[axis] = accLPF32[axis]>>ACC_LPF_FACTOR;
    accMag += (long)imu.accSmooth[axis] * imu.accSmooth[axis]; // X^2 + Y^2 + Z^2
  }
  rotateV( &EstA.V, deltaGyroAngle ); //旋转估计 三轴加速度值
  rotateV( &EstM.V, deltaGyroAngle ); //旋转估计 三轴磁力计值
  accMag = (accMag*100) >> 18; /// ( (long)ACC_1G * ACC_1G ); // = (*g)^2 * 100         
  validAcc = 72 < (int)accMag && (int)accMag < 133; // if accel: >(1.15g)^2 或 <(0.85g)^2 ；则 不进行complimentart filter(Gyro drift corrention)
//           USART_Transmit('0'+validAcc);
  for(axis = 0; axis < 3; axis++){//Apply complimentart filter ??(Gyro drift corrention)??
    if(validAcc)
      EstA.A[axis] = ( EstA.A[axis] * GYR_CMPFA_FACTOR + imu.accSmooth[axis] ) * INV_GYR_CMPFA_FACTOR; /*imu.accADC[axis];*/
    EstA32.A[axis] = (long)EstA.A[axis];
     
    EstM.A[axis] = ( EstM.A[axis] * GYR_CMPFM_FACTOR + imu.magADC[axis] ) * INV_GYR_CMPFM_FACTOR; /*imu.magADC[axis];*/
    EstM32.A[axis] = (long)EstM.A[axis];
  }
  
  if( (int)EstA.A[2]/*??*/ > ACCZ_25deg )
    flags.SMALL_ANGLE_25 = 1;
  else 
    flags.SMALL_ANGLE_25 = 0;
  // Attitude of the estimated vector
  long sqGX_sqGZ = EstA32.V.X * EstA32.V.X + EstA32.V.Z * EstA32.V.Z; // X^2 + Z^2
  invG = InvSqrt( sqGX_sqGZ + EstA32.V.Y * EstA32.V.Y );
  att.angle[ROLL] = _atan2( EstA32.V.X, EstA32.V.Z ); // -1800deg -> 0 -> 1800deg
  att.angle[PITCH] = _atan2( EstA32.V.Y, InvSqrt(sqGX_sqGZ)*sqGX_sqGZ/* (sqGX_sqGZ)^0.5 */); // -900deg -> 0 -> 900deg
  
  att.heading = _atan2( EstM32.V.Z * EstA32.V.X - EstM32.V.X * EstA32.V.Z, //???
                       ( EstM.V.Y * sqGX_sqGZ - (EstM32.V.X * EstA32.V.X + EstM32.V.Z * EstA32.V.Z) * EstA.V.Y ) * invG );
  //att.heading += conf.mag_declination; //ding xiang
  att.heading /= 10; //缩放 [-180, 180]
} 