#include "MAIN.h"

void initI2C(void)//400kHZ 16MHZ主频
{
  //预分频TWPS=1
  TWBR=0X03;//TWBR=3
}
unsigned char I2C_Write(unsigned char DeviceAddrW,unsigned char RegAddr,unsigned char data)
{
  Start();
  Wait();
  if(TestAck()!=START)
    return 1;
  
  Write8Bits(DeviceAddrW);
  Wait();
  if(TestAck()!=MT_SLA_ACK)
    return 1;
  
  Write8Bits(RegAddr);
  Wait();
  if(TestAck()!=MT_DATA_ACK)
    return 1;
  
  Write8Bits(data);
  Wait();
  if(TestAck()!=MT_DATA_ACK)
    return 1;
  Stop();
 // Delay_ms(5);
  return 0;
}

unsigned char I2C_Read(unsigned char DeviceAddrW,unsigned char DeviceAddrR,unsigned char RegAddr)
{
  unsigned char temp;
  Start();
  Wait();
  if(TestAck()!=START)
    return 1;
  
  Write8Bits(DeviceAddrW);
  Wait();
  if(TestAck()!=MT_SLA_ACK)
    return 1;
  
  Write8Bits(RegAddr);
  Wait();
  if(TestAck()!=MT_DATA_ACK)
    return 1;
  
  Start();
  Wait();
  if(TestAck()!=RE_START)
    return 1;
  
  Write8Bits(DeviceAddrR);    //启动主I2C读方式
  Wait();
  if(TestAck()!=MR_SLA_ACK)
    return 1;
  
  Twi();  //开始读数据
  Wait();
  if(TestAck()!=MR_DATA_NOACK)
    return 1;
  
  temp=TWDR;
  Stop();
  return temp;
}

//void AuxiliaryI2C_Write(unsigned char Addr,unsigned char data)
//{
//  
//}
/******  from MWC *******/
//void i2c_init(void){
//  TWST = 0;
//  TWBR = ( (F_CPU /I2C_SPEED) - 16 ) /2;
//  TWCR = 1<<TWEN:
//}

void waitTransmissionI2C(void){
  unsigned char count = 255;
  while( !(TWCR & (1<<TWINT)) ){
    count--;
    if(count == 0){
      TWCR = 0;
      /* Error processing*/
      break;
    }
  }
}

void i2c_stop(void){
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

void i2c_rep_start(unsigned char address){
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  waitTransmissionI2C();
  TWDR = address;
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

void i2c_write(unsigned char data){
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

unsigned char i2c_read(unsigned char ack){
  TWCR = (1<<TWINT) | ( 1<<TWEN) | (ack ? (1<<TWEA) : 0); // mpu6050 product specification P36
  waitTransmissionI2C();
  unsigned char r = TWDR;
  if(!ack) i2c_stop();
  return r;
}

size_t i2c_read_to_buf( unsigned char add, void *buf, size_t size ){
  i2c_rep_start( (add<<1) | 1 ); // add = addresss + R(1); W(0)
  size_t bytes_read = 0;
  unsigned char *b = (unsigned char*)buf;
  while(size--){
    *b++ = i2c_read( size>0 ); // the last one reply with Nck
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf( unsigned char add, unsigned char reg, void *buf, size_t size ){
  i2c_rep_start( add<<1 ); // add + write
  i2c_write( reg );
  return i2c_read_to_buf( add, buf, size);
}

void i2c_writeReg( unsigned char add, unsigned char reg, unsigned char val){
  i2c_rep_start(add<<1);
  i2c_write(reg);
  i2c_write(val);
  i2c_stop();
}

unsigned char i2c_readReg(unsigned char add, unsigned char reg){
  unsigned char val;
  i2c_read_reg_to_buf(add,reg, &val, 1);
  return val;
}