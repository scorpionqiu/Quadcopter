#define Start() TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN)//����START�ź�
#define Stop()  TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO)//����STOP�ź�
#define Wait()  while(!(TWCR&(1<<TWINT)));  //�ȴ�TWINT��λ����ʾSTART�źŷ���
#define TestAck() TWSR&0xf8    //����״̬
#define SetAck()  TWCR|=(1<<TWEA)    //����Ӧ���ź�
#define SetNoAck() TWCR&=~(1<<TWEA) 
#define Twi() TWCR=(1<<TWINT)|(1<<TWEN) //����I2C
#define Write8Bits(x) {TWDR=(x);TWCR=(1<<TWINT)|(1<<TWEN);} //TWINTд1��0

#define START 0x08
#define RE_START 0x10
#define MT_SLA_ACK 0x18
#define MT_DATA_ACK 0x28
#define MR_SLA_ACK 0x40
#define MR_DATA_NOACK 0x58

unsigned char I2C_Write(unsigned char,unsigned char,unsigned char);
unsigned char I2C_Read(unsigned char,unsigned char,unsigned char);
void initI2C(void);

size_t i2c_read_reg_to_buf( unsigned char add, unsigned char reg, void *buf, size_t size );
void i2c_writeReg(unsigned char, unsigned char, unsigned char);
unsigned char i2c_readReg(unsigned char , unsigned char);