#include"MAIN.h"
#define TX_BUFFER_SIZE 64 //
static volatile unsigned char serialTxInputPos, serialTxOutputPos;
static unsigned char serialTxBuffer[TX_BUFFER_SIZE];
static unsigned char checkSum; // check sum/code
       unsigned char cmd;

void initUSART(unsigned long baud)//�첽ͨ�� 16Mʱ�� baud 9600 14400 19200 28800 38400 57600 76800 115200
{
  unsigned char ubrr;
  if(baud==9600  ) ubrr = 103;
  if(baud==14400 ) ubrr = 68 ;
  if(baud==19200 ) ubrr = 51 ;
  if(baud==28800 ) ubrr = 34 ;
  if(baud==38400 ) ubrr = 25 ;
  if(baud==57600 ) ubrr = 16 ;
  if(baud==76800 ) ubrr = 12 ;
  if(baud==115200) ubrr = 8  ;
  /*���ò�����*/
  UBRR0H=(unsigned char)(ubrr>>8);
  UBRR0L=(unsigned char)ubrr; 
  /*���ͼ�����ʹ��*/
  UCSR0B=(1<<RXEN0)|(1<<TXEN0)/*RX complete interupt enable |(1<<RXCIE0)*/;
  /*��������֡��ʽ*/
  UCSR0C=(0<<UMSEL00)|(0<<USBS0)|(3<<UCSZ00);   //The default is fine
     //�첽��������У��UPM=00��1ֹͣλ��8λ������T �½�R(UCPOL=0)
}

void UartSendData(void){ //start to trasmit data
  UCSR0B |= (1<<UDRIE0); // Enable trasmitter UDRE interrupt
}

#pragma vector = USART_UDRE_vect //#define  USART_UDRE_vect      (0x4C) /* USART, Data Register Empty */
__interrupt void SendData(void){
  unsigned char t = serialTxOutputPos;
  if( serialTxInputPos != t ){
    if( ++t >= TX_BUFFER_SIZE ) t = 0;
    UDR0 = serialTxBuffer[t];
    serialTxOutputPos = t;
  }
  if( t == serialTxInputPos ) UCSR0B &= ~(1<<UDRIE0); // Data buffer is transmitted completely, disable transmitter UDRE interrupt
}

void SerialSerialize(unsigned char a){
  unsigned char t = serialTxInputPos;
  if( ++t >= TX_BUFFER_SIZE ) t = 0;
  serialTxBuffer[t] = a;
  serialTxInputPos = t;
}

void SerialWrite(unsigned char c){
  SerialSerialize(c);
  UartSendData();
}

void serialize8(unsigned char a){
  SerialSerialize(a);
  checkSum ^= a;
}
void serialize16(int a){ // Low first, high later
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void headSerialResponse(unsigned char s){ // Data package header { $ M > dataNum cmd}
  serialize8('$');
  serialize8('M');
  serialize8('>');
  checkSum = 0; // start a new check sum
  serialize8(s);
  serialize8(cmd);
}
//void headSerialReply(unsigned char s){
//  headSerialResponse(s);
//}
void tailSerialReply(void){ // Add check sum , then start to send
  serialize8(checkSum);
  UartSendData();
}

void s_struct(unsigned char *cb, unsigned char siz){
  headSerialResponse(siz);
  while(siz--) serialize8(*cb++);
}//OUTPUT:  $ M > charCnt cmd data (+checksum)
void evaluateCommand(){
  static unsigned char sendThis = 0;
  switch( sendThis ){
  case 0:
    sendThis = 1;
    cmd = MSP_RAW_IMU;
    
//    imu.gyroData[0] = temp;
    
    s_struct((unsigned char*)&imu, 18); // accADC[]; gyroData[2](instead of gyroADC[]); magADC[]
    break;
  case 1:
    sendThis = 2;
    cmd = MSP_ATTITUDE;
    s_struct((unsigned char*)&att, 6); // roll; pitch; yaw(heading)
    break;
  case 2:
    sendThis = 0;
    cmd = MSP_RC;
    s_struct((unsigned char*)&rcData, 8); // rcRoll; rcPitch; rcYaw; rcThrottle
    break;
//  case 3:
//    sendThis = 0;
//    cmd = 
  }
  tailSerialReply();
 }

void SerialCom(void){
  
  /* code about processing cmd from GUI */
  
  evaluateCommand();
}

/************** below: old code ********************/
//void USART_Transmit(unsigned char data) //78us; theoretically 69us
//{
//  /*�ȴ����ͻ�����Ϊ��*/
//  while(!(UCSR0A&(1<<UDRE0)))//UDREn is cleared by writing UDRn
//    ;
//  UDR0=data;
////  while(!(UCSR0A&(1<<TXC0)));
////  UCSR0A &= ~(1<<TXC0);  
//}
//
//unsigned char USART_Receive(void)
//{
//  /*�ȴ���������*/
//  while(!(UCSR0A&(1<<RXC0)))
//    ;
//  /*�ӻ������л�ȡ����������*/
//  return UDR0;
//}
//
//void Send_intData(int _int) //547us
//{
//  unsigned char SingleNum;
//        if(_int<0){_int=-_int;USART_Transmit('-');}
//        else 
//          USART_Transmit(' ');
//  SingleNum=_int/10000;
//  USART_Transmit(SingleNum+'0');
//  _int%=10000;
//  SingleNum=_int/1000;
//  USART_Transmit(SingleNum+'0');
//  _int%=1000;
//  SingleNum=_int/100;
//  USART_Transmit(SingleNum+'0'); 
//  _int%=100;
//  SingleNum=_int/10;
//  USART_Transmit(SingleNum+'0');
//    _int%=10;
//  SingleNum=_int;
//  USART_Transmit(SingleNum+'0');
//}
//void Send_longData(long _long){ //859us
//  unsigned char single;
//  if( _long < 0 ) {_long = -_long; USART_Transmit('-');}
//  else USART_Transmit(' ');
//  single = _long/1000000000;
//  USART_Transmit(single+'0');
//  _long %= 1000000000;
//  single = _long/100000000;
//  USART_Transmit(single+'0');
//  _long %= 100000000;
//  single = _long/10000000;
//  USART_Transmit(single+'0');
//  _long %= 10000000;
//  single = _long/1000000;
//  USART_Transmit(single+'0');
//  _long %= 1000000;
//  single = _long/100000;
//  USART_Transmit(single+'0');
//  _long %= 100000;
//  single = _long/10000;
//  USART_Transmit(single+'0');
//  _long %= 10000;
//  single = _long/1000;
//  USART_Transmit(single+'0');
//  _long %= 1000;
//  single = _long/100;
//  USART_Transmit(single+'0');
//  _long %= 100;
//  single = _long/10;
//  USART_Transmit(single+'0');
//  _long %= 10;
//  single = _long/1;
//  USART_Transmit(single+'0');
//}
//void SendUlong2Float(unsigned long _ulong)
//{
//  float t = (float)_ulong/1000000;  // microseconds to seconds
//  int  t1 = (int)t;
//  Send_intData(t1);
//  USART_Transmit('.');
//  t1 = (int)( (t-t1)*10000 );
//  Send_intData(t1);
//  USART_Transmit('\n');
//}
//void SendFloat
/************** above: old code ********************/