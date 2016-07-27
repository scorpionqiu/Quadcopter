#include "MAIN.h"
unsigned char rawADC[6];
void getADC(void);
void MPU6050_Init(void)
{
  I2C_Write(WD_DEVICE_ADDR, PWR_MGMT_1, 0X80); // DEVICE_RESET  1
  delay_ms(50);
  I2C_Write(WD_DEVICE_ADDR, PWR_MGMT_1, 0X03);//PWR_MGMT_1: SLEEP 0; CYCLE 0; TEMP_DIS 0;
                                              // CLKSEL 3 (PLL with Z Gyro reference)
//  I2C_Write(WD_DEVICE_ADDR, SMPLRT_DIV, 0X07);//陀螺仪125HZ采样 , commented means that 0x00 1kHz
  I2C_Write(WD_DEVICE_ADDR, CONFIG, 0X04);//DLP_CFG=4 :低通滤波20HZ FSYNC端口已接地 
  I2C_Write(WD_DEVICE_ADDR, GYRO_CONFIG, 0X18);//FS_SEL=3 :2000度/sec，无自检
  I2C_Write(WD_DEVICE_ADDR, ACCEL_CONFIG, 0X10);//8g，无自检 
       //master mode  to magnetic compass
#ifdef _MASTER_MODE_
  ////////
#endif
}

void i2c_getSixRawADC(unsigned char add, unsigned char reg){
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

static float magGain[3] = {1.0, 1.0, 1.0}; // populated at sensor init
static unsigned char magInit = 0;
void HMC5883L_Init(void)
{
#ifdef _BYPASS_MODE_
  I2C_Write(WD_DEVICE_ADDR, INT_PIN_CFG, 0x02);  //enable bypass mode
  I2C_Write(WD_DEVICE_ADDR, USER_CTRL, 0x00);    //disable master mode
#endif
#ifdef _MASTER_MODE_
/*  *   */  
#endif
  long xyz_total[3] = {0,0,0};
  bool bret = true; //error indicator
  delay_ms(10);
  I2C_Write(WD_CMP_ADDR, CRA, 0x10+0x01);// Input Rate: 30Hz; Positive bias configuration for X,Y,Z
  I2C_Write(WD_CMP_ADDR, CRB, 2<<5); // Set the Gain: 010 1.9Ga   * 820 Counts/Gauss
  I2C_Write(WD_CMP_ADDR, MR, 0x01);    // Single-measurement mode for test
  delay_ms(100);
  getADC(); //Get one sample, and discard it
  
  for(unsigned char i = 0; i < 10; i++){
    I2C_Write(WD_CMP_ADDR, MR, 0x01);
    delay_ms(100);
    getADC(); 
    xyz_total[0] += imu.magADC[0];
    xyz_total[1] += imu.magADC[1];
    xyz_total[2] += imu.magADC[2];
    // Detect saturation
    if( -(1<<12) >= _min(imu.magADC[0], _min(imu.magADC[1], imu.magADC[2])) ){
      bret = false;
      break;
    }
  }
  //apply the negative bias. others same again
  I2C_Write(WD_CMP_ADDR, CRA, 0x10+0x02); // negative bias
  for(unsigned char i = 0; i < 10; i++){
    I2C_Write(WD_CMP_ADDR, MR, 0x01);
    delay_ms(100);
    getADC(); 
    xyz_total[0] -= imu.magADC[0];
    xyz_total[1] -= imu.magADC[1];
    xyz_total[2] -= imu.magADC[2];
    // Detect saturation
    if( -(1<<12) >= _min(imu.magADC[0], _min(imu.magADC[1], imu.magADC[2])) ){
      bret = false;
      break;  
    }
  }
  magGain[0] = fabs(820.0 * X_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[0]);
  magGain[1] = fabs(820.0 * Y_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[1]);
  magGain[2] = fabs(820.0 * Z_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[2]);
  // leave test mode
  I2C_Write(WD_CMP_ADDR, CRA, 0x70); //0 11 100 00 8_samples ; 15Hz; no bias
  I2C_Write(WD_CMP_ADDR, CRB, 0x20); // 001 00000  Gain: 1.3 Ga    * 1090 Counts/Gauss
  I2C_Write(WD_CMP_ADDR, MR,  0x00); // continuous_measurement mode
  delay_ms(100);
  magInit = 1;
  
  if(!bret){ //something went wrong so get a best guess
   magGain[0] = 1.0;
   magGain[1] = 1.0;
   magGain[2] = 1.0;
  }
}

void Acc_getADC(void) // 188us
{
//  rawADC[0]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, ACCEL_XOUT_H);
//  rawADC[1]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, ACCEL_XOUT_L);
//  rawADC[2]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, ACCEL_YOUT_H);
//  rawADC[3]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, ACCEL_YOUT_L);
//  rawADC[4]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, ACCEL_ZOUT_H);
//  rawADC[5]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, ACCEL_ZOUT_L);
  i2c_getSixRawADC( mpu6050_add, 0x3B );
  ACC_ORIENTATION( (rawADC[0]<<8 | rawADC[1])>>3, //to imu.accADC[3]
                   (rawADC[2]<<8 | rawADC[3])>>3,
                   (rawADC[4]<<8 | rawADC[5])>>3 ); //缩小输出(+\-): 32768/8 =4096, max: 8g
  static long a[3];
  unsigned char axis;
  if(calibratingA > 0){ // calibrating Acc( global_conf.accZero[3] ) at the beginning
    for(axis = 0; axis < 3; axis++){
      if(calibratingA == 512) a[axis] = 0; //Reset a[axis] at start of calibration
      a[axis] += imu.accADC[axis]; // Sum up 512 reading
      imu.accADC[axis] = 0;
      global_conf.accZero[axis] = 0;
    }
    // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
    if(calibratingA == 1){ // after sum up 512 reading
      global_conf.accZero[ROLL] = (a[ROLL] + 256) >>9; // ? +256 ?   2^9 = 512
      global_conf.accZero[PITCH]= (a[PITCH] + 256) >>9;
      global_conf.accZero[YAW]  = ((a[YAW] + 256) >>9) - ACC_1G;
      conf.angleTrim[ROLL] = 0; // used for what ??
      conf.angleTrim[PITCH] = 0;
      writeGlobalSet(1); //write accZero to EEPROM 
    }
    calibratingA--; //512->1
  }
#if defined( INFLIGHT_ACC_CALIBRATION )
  static long b[3];
  static int accZero_saved[3] = {0, 0, 0};
  static int angleTrim_saved[2] = {0, 0};
  //Saving old zeroPoints before measurement
  if( inflightCalibratingA == 50 ){
    accZero_saved[ROLL] = global_conf.accZero[ROLL];
    accZero_saved[PITCH] = global_conf.accZero[PITCH];
    accZero_saved[YAW] = global_conf.accZero[YAW];
    angleTrim_saved[ROLL] = conf.angleTrim[ROLL];
    angleTrim_saved[PITCH] = conf.angleTrim[PITCH];
  }
  if(inflightCalibratingA > 0){
    for(axis = 0; axis < 3; axis++){
      if(inflightCalibratingA == 50) b[axis] = 0;
      b[axis] += imu.accADC[axis];
      //clear global variables for next reading
      imu.accADC[axis] = 0;
      global_conf.accZero[axis] = 0;
    }
    if(inflightCalibratingA == 1){ // The end of calibrating
      accInflightCalibrationActive = 0;
      accInflightCalibrationMeasurementDone = 1;
      //recover saved values to maintain current flight behavior until new values are transferred
      for(axis = 0; axis<3; axis++){
        global_conf.accZero[axis] = accZero_saved[axis];
        if(axis <2) 
          conf.angleTrim[axis] = angleTrim_saved[axis];
      }
    }
    inflightCalibratingA--;
  }
  // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
  if(accInflightCalibrationSavetoEEProm == 1){ // th copter is landed, disarmed and the combo has been done again
    accInflightCalibrationSavetoEEProm = 0;
    global_conf.accZero[ROLL] = b[ROLL]/50;
    global_conf.accZero[PITCH]= b[PITCH]/50;
    global_conf.accZero[YAW]  = b[YAW]/50 - ACC_1G;
    conf.angleTrim[ROLL] = 0;
    conf.angleTrim[PITCH]= 0;
    writeGlobalSet(1); // write accZero to EEPPROM
  } 
#endif
  //This is the ultimate imu.accADC[]
  imu.accADC[ROLL] -= global_conf.accZero[ROLL]; 
  imu.accADC[PITCH]-= global_conf.accZero[PITCH];
  imu.accADC[YAW]  -= global_conf.accZero[YAW];
#if defined(SENSORS_TILT_45deg_LEFT)
  /*****/
#endif
#if defined(SENSORS_TILT_45deg_RIGHT)
  /*****/
#endif
}

void Gyro_getADC(void) // 204us
{
//  rawADC[0]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, GYRO_XOUT_H);
//  rawADC[1]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, GYRO_XOUT_L);
//  rawADC[2]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, GYRO_YOUT_H);
//  rawADC[3]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, GYRO_YOUT_L);
//  rawADC[4]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, GYRO_ZOUT_H);
//  rawADC[5]=I2C_Read(WD_DEVICE_ADDR, RD_DEVICE_ADDR, GYRO_ZOUT_L);
  i2c_getSixRawADC( mpu6050_add, 0x43 );
  GYRO_ORIENTATION( (rawADC[0]<<8 | rawADC[1])>>2, //to imu.gyroADC[3];
                    (rawADC[2]<<8 | rawADC[3])>>2,
                    (rawADC[4]<<8 | rawADC[5])>>2 ); // 缩小输出(+\-): 32768/4 =8192, max: 2000deg/sec
  static int previousGyroADC[3] = {0, 0, 0};
  static long g[3];
  unsigned char axis, tilt = 0;
#if defined MMGYRO
  // Moving Average Gyros by Magnetron1
  
#endif 
  if(calibratingG > 0){
    for(axis = 0; axis < 3; axis++){
    // Reset g[axis] at start of calibration
      if(calibratingG == 512){
        g[axis] = 0;
            #if defined(GYROCALIBRATIONFAILSAFE)
              previousGyroADC[axis] = imu.gyroADC[axis];
            }
            if(calibratingG % 10 == 0){
              if(fabs(imu.gyroADC[axis]) - previouGyroADC[axis]) > 0) tilt = 1;
              previousGyroADC[axis] = imu.gyroADC[axis];
            #endif
      }
      //Sum up 512 reading
      g[axis] += imu.gyroADC[axis];
      // Clear global variables for next reading
      imu.gyroADC[axis] = 0;
      if(calibratingG == 1) {// The end of calibratingG
        gyroZero[axis] = ( g[axis] + 256 )>>9;
        //if(axis == 2) ledTwinkle(30); //GYRO calibration complete
      }
    }
  #if defined(GYROCSLIBRSTIONFAILSAFE)
    if(tilt) { calibratingG = 1000; /* LED_ON; */ } 
    else     { calibratingG--;   /* LED_OFF */ }
  #else
    calibratingG--;
  #endif
  }
  #ifdef MMGYRO
    /* as regards codes */
  #else
  for(axis = 0; axis < 3; axis++){
    imu.gyroADC[axis] -= gyroZero[axis];
    // This is the ultimate imu.gyroADC[]
    imu.gyroADC[axis] = CONSTRAIN(imu.gyroADC[axis], previousGyroADC[axis]-800, previousGyroADC[axis]+800); 
  #endif
    previousGyroADC[axis] = imu.gyroADC[axis];
  }
#if defined(SENSORS_TILT_45deg_LEFT)
  /*  */
#endif
#if defined(SENSORS_TILT_45deg_RIGHT)
  /*  */
#endif
}

unsigned char Mag_getADC(void) // new 336us (<350us)
{
  static unsigned long t, tCal = 0;
  static int magZeroTempMin[3];
  static int magZeroTempMax[3];
  unsigned char axis;
  currentTime = micros();
  if( currentTime < t )  return 0;
  t = currentTime + 100000; // each read is spaced by 100ms
//Start to read mag_data  
  getADC();
// Start to process mag_data from reading
  imu.magADC[ROLL] = (int)( imu.magADC[ROLL]  * magGain[ROLL] );
  imu.magADC[PITCH]= (int)( imu.magADC[PITCH] * magGain[PITCH] );
  imu.magADC[YAW]  = (int)( imu.magADC[YAW]   * magGain[YAW] );
  
//      for(char i = 0; i < 3; i++){
//      _TEMP[i] = magGain[i]*100;
//    }
  
  if(flags.CALIBRATE_MAG){
    tCal = t;
    for(axis = 0; axis < 3; axis++){
      global_conf.magZero[axis] = 0;// ???????
      magZeroTempMin[axis] = imu.magADC[axis];
      magZeroTempMax[axis] = imu.magADC[axis];
    }
    flags.CALIBRATE_MAG = 0; // Mag calibrate completed
  }
  if( magInit ) {// offset only once mag calibration is done
    imu.magADC[ROLL] -= global_conf.magZero[ROLL];
    imu.magADC[PITCH]-= global_conf.magZero[PITCH];
    imu.magADC[YAW]  -= global_conf.magZero[YAW];
  }
  if(tCal != 0){
    if( (t - tCal) < 30000000 ) { // 30s to turn the muti in all directions  旋转校准
      LED_TOGGLE;
      for(axis = 0; axis < 3; axis++){
        if( imu.magADC[axis] < magZeroTempMin[axis] ) magZeroTempMin[axis] = imu.magADC[axis];
        if( imu.magADC[axis] > magZeroTempMax[axis] ) magZeroTempMax[axis] = imu.magADC[axis];
      }
    } else{
      tCal = 0;
      for( axis = 0; axis < 3; axis++)
        global_conf.magZero[axis] = ( magZeroTempMin[axis] + magZeroTempMax[axis] ) >>1;
      writeGlobalSet(1); //write magZero to EEPROM
    }
  }
  else{
  #if defined(SENSORS_TILT_45deg_LEFT)
      
  #endif
  #if defined(SENSORS_TILT_45deg_RIGHT)
      
  #endif
  }
  return 1;
}
void getADC(void){
//  rawADC[0] = I2C_Read(WD_CMP_ADDR, RD_CMP_ADDR, 0x03);//x  > 380us
//  rawADC[1] = I2C_Read(WD_CMP_ADDR, RD_CMP_ADDR, 0x04); 
//  rawADC[4] = I2C_Read(WD_CMP_ADDR, RD_CMP_ADDR, 0x05);//z
//  rawADC[5] = I2C_Read(WD_CMP_ADDR, RD_CMP_ADDR, 0x06);    
//  rawADC[2] = I2C_Read(WD_CMP_ADDR, RD_CMP_ADDR, 0x07);//y
//  rawADC[3] = I2C_Read(WD_CMP_ADDR, RD_CMP_ADDR, 0x08);
  i2c_getSixRawADC( hmc5883l_add, 0x03 );  // 176us

  MAG_ORIENTATION( (rawADC[0]<<8 | rawADC[1]),
                   (rawADC[4]<<8 | rawADC[5]),
                   (rawADC[2]<<8 | rawADC[3]) );
}

void initSensors(void){
  MPU6050_Init();
  HMC5883L_Init();
}