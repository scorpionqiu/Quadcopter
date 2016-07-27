
#define SMPLRT_DIV   0X19 //设置陀螺仪采样率
#define CONFIG       0X1A
#define GYRO_CONFIG  0X1B
#define ACCEL_CONFIG 0X1C
#define INT_PIN_CFG  0X37
#define ACCEL_XOUT_H 0X3B
#define ACCEL_XOUT_L 0X3C
#define ACCEL_YOUT_H 0X3D
#define ACCEL_YOUT_L 0X3E
#define ACCEL_ZOUT_H 0X3F
#define ACCEL_ZOUT_L 0X40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0X42
#define GYRO_XOUT_H  0X43
#define GYRO_XOUT_L  0X44
#define GYRO_YOUT_H  0X45
#define GYRO_YOUT_L  0X46
#define GYRO_ZOUT_H  0X47
#define GYRO_ZOUT_L  0X48
#define I2C_SLV0_ADDR 0X25
#define I2C_SLV0_REG  0X26
#define I2C_SLV0_CTRL 0X27
#define INT_PIN_CFG  0X37  //设置 I2C_BYPASS_EN 0x02
#define USER_CTRL    0X6A  //设置 I2C_MST_EN
#define PWR_MGMT_1   0X6B
#define WHO_AM_I     0X75  //默认值0x68，最高位和最低位无效，板子上AD0接地

#define WD_DEVICE_ADDR 0XD0  //MPU6050设备地址+写 0x1101 0000
#define RD_DEVICE_ADDR 0XD1  //MPU6050设备地址+读
  #define mpu6050_add 0x68  // 0x0111 0000

#define I2C_SLV0_ADDR 0X25
#define I2C_SLV0_REG 0X26
#define I2C_SLV0_CTRL 0X27
#define I2C_SLV0_DO 0X63

#define CRA   0X00
#define CRB   0X01
#define MR    0X02
#define DOXRH 0X03
#define DOXRL 0X04
#define DOZRH 0X05
#define DOZRL 0X06
#define DOYRH 0X07
#define DOYRL 0X08
#define WD_CMP_ADDR 0X3C // 0x0011 1100
#define RD_CMP_ADDR 0X3D // 0x0011 1101
  #define hmc5883l_add 0x1E // 0x0001 1110
#define X_SELF_TEST_GAUSS 1.16 
#define Y_SELF_TEST_GAUSS 1.16
#define Z_SELF_TEST_GAUSS 1.08

#define ACC_1G 512  // 8g 4096 LSB/g  same as Acc_getADC(): >>3
#define ACCZ_25deg (int)ACC_1G*0.90631 //小角度旋转限制; cos(25) = 0.90631
#define GYRO_SCALE ( 4 / 16.4 * 3.14 / 180 / 1000000 ) // 16.4LSB/(deg/s); /1000000(us->s); 4 ( according to Gyro_getADC(): >>2 );

#define ACC_ORIENTATION(X, Y, Z) {imu.accADC[ROLL] = -X; imu.accADC[PITCH] = -Y; imu.accADC[YAW] = Z;} //符号???? - - +
#define GYRO_ORIENTATION(X,Y, Z) {imu.gyroADC[ROLL] = Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}  //     + - -
#define MAG_ORIENTATION(X, Y, Z) {imu.magADC[ROLL] = X; imu.magADC[PITCH] = Y; imu.magADC[YAW] = -Z;}  //       + + -

void initSensors(void);
void Acc_getADC(void);
void Gyro_getADC(void);
unsigned char Mag_getADC(void);
void getADC(void);
