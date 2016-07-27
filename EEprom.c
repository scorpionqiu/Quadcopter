#include "MAIN.h"
//unsigned char eeprom_read_byte( unsigned int add ){
//  while(EECR & (1<<EEPE)) ;
//  EEAR = add;
//  EECR |= (1<<EERE);
//  return EEDR;
//}
//
//void eeprom_write_byte(unsigned int add, unsigned char data){
//  while(EECR & (1<<EEPE)) ;
//  EEAR = add;
//  EEDR = data;
//  EECR |= (1<<EEMPE);
//  EECR |= (1<<EEPE);
//}

#define offset 0//8

void eeprom_read_block( void* val, void* add, unsigned char size ){
  unsigned char *v = val;
  unsigned int  *a = add;
  //while(size--) __EEGET( *(v++), a++ ); 
  for(unsigned char i = 0; i < size; i++ ){
    __EEGET( *(v+i), a+i);
  }
//  __EEGET( *v, a);__EEGET( *(v+1), a+1 ); __EEGET(*(v+2), a+2 );
//  __EEGET( *(v+size-1), a+size-1 );
}

void eeprom_write_block( void* val, void* add, unsigned char size){
  unsigned char *v = val;
  unsigned int  *a = add;
  //while(size--) __EEPUT( a++, *(v++) );     //eeprom_write_byte( *a++, *(v++) );
  for( unsigned char i = 0; i< size; i++){
    __EEPUT( a+i, *(v+i));
  }
//  __EEPUT( a, *v);__EEPUT( a+1, *(v+1)); __EEPUT(a+2, *(v+2));
//  __EEPUT( a+size-1, *(v+size-1));
}

//void eeprom_write_block1( void *val, unsigned int add, unsigned char size){
//  unsigned int a = add;
//  unsigned char* v = val;
//  while(size--) eeprom_write_byte(a++, *(v++));
//}

void LoadDefaults(void){
#if  PID_CONTROLLER == 1
  conf.pid[ROLL].P8 = 33; conf.pid[ROLL].I8 = 30; conf.pid[ROLL].D8 = 23;
  conf.pid[PITCH].P8 = 33; conf.pid[PITCH].I8 = 30; conf.pid[PITCH].D8 = 23;
  conf.pid[PIDLEVEL].P8 = 90; conf.pid[PIDLEVEL].I8 = 10; conf.pid[PIDLEVEL].D8 = 100;
#elif PID_CONTROLLER == 2
  conf.pid[ROLL].P8 = 128/*28*/; conf.pid[ROLL].I8 = 10; conf.pid[ROLL].D8 = 128/*7*/;
  conf.pid[PITCH].P8 = 128/*28*/; conf.pid[PITCH].I8 = 10; conf.pid[PITCH].D8 = 128/*7*/;
  conf.pid[PIDLEVEL].P8 = 30; conf.pid[PIDLEVEL].I8 = 32; conf.pid[PIDLEVEL].D8 = 0;
#endif
  conf.pid[YAW].P8 = 68; conf.pid[YAW].I8 = 45; conf.pid[YAW].D8 = 0;
  conf.pid[PIDALT].P8 = 68; conf.pid[PIDALT].I8 = 25; conf.pid[PIDALT].D8 = 24;
  
//  conf.pid[PIDPOS].P8  = POSHOLD_P * 100;     conf.pid[PIDPOS].I8    = POSHOLD_I * 100;       conf.pid[PIDPOS].D8    = 0;
//  conf.pid[PIDPOSR].P8 = POSHOLD_RATE_P * 10; conf.pid[PIDPOSR].I8   = POSHOLD_RATE_I * 100;  conf.pid[PIDPOSR].D8   = POSHOLD_RATE_D * 1000;
//  conf.pid[PIDNAVR].P8 = NAV_P * 10;          conf.pid[PIDNAVR].I8   = NAV_I * 100;           conf.pid[PIDNAVR].D8   = NAV_D * 1000;
  conf.pid[PIDMAG].P8   = 40;
//  conf.pid[PIDVEL].P8 = 0;      conf.pid[PIDVEL].I8 = 0;    conf.pid[PIDVEL].D8 = 0;
  
  conf.rcRate8 = 90;///define value range
  conf.rcExpo8 = 65;
  conf.rollPitchRate = 0;
  conf.yawRate = 0;
  conf.dynThrPID = 0;
  conf.thrMid8 = 50; conf.thrExpo8 = 0;
  
  update_constants();
}

unsigned char calculate_sum(unsigned char *cb, unsigned char siz){
  unsigned char sum = 0x55;
  while(--siz) sum += *cb++; // ignore the last one: check
  return sum;
}

void readGlobalSet(void){
  eeprom_read_block( (void*)&global_conf, (void*)0, sizeof(global_conf) );
  if( calculate_sum((unsigned char*)&global_conf, sizeof(global_conf)) != global_conf.checksum ){
    global_conf.currentSet = 0;
    global_conf.currentSet = 0;
    global_conf.accZero[ROLL] = 5000; // for config error signalization
  }
}

void writeGlobalSet(unsigned char b){
  global_conf.checksum = calculate_sum((unsigned char*)&global_conf, sizeof(global_conf));
  eeprom_write_block( (void*)&global_conf, (void*)0, sizeof(global_conf) );
  //eeprom_write_block1( (void*)&global_conf, 0, sizeof(global_conf) );
  if(b == 1) ledTwinkle(20); // writeGlobalSet complete
}

bool readEEPROM(){
  unsigned char i;
  eeprom_read_block( (void*)&conf, (void*)(offset+sizeof(global_conf)), sizeof(conf) );
  
  if( calculate_sum((unsigned char*)&conf, sizeof(conf)) != conf.checksum ){
    ledSOS(5);  // fail to read conf
    LoadDefaults();
    return false;
  } 
//  conf.rcExpo8 = 65;conf.rcRate8 = 90; 
  for(i=0; i<5; i++){
    lookupPitchRollRC[i] = (1526+conf.rcExpo8*(i*i-15)) * i * (long)conf.rcRate8 /1192;
  }
  for(i=0; i<11; i++){
    int tmp = 10*i -conf.thrMid8;
    unsigned char y = 1;
    if(tmp>0) y = 100 - conf.thrMid8;
    if(tmp<0) y = conf.thrMid8;
    lookupThrottleRC[i] = 10*conf.thrMid8 + tmp * ( 100-conf.thrExpo8+(long)conf.thrExpo8*(tmp*tmp)/(y*y) ) /10;
    lookupThrottleRC[i] = conf.minthrottle + (long)(MAXTHROTTLE-conf.minthrottle)* lookupThrottleRC[i]/1000;
  }
  
  /*  produce:  RC channel curve adjust factor  */
  /*  and others need to implement   */
  
  return 1;
}

void writeParams(unsigned char b){
  conf.checksum = calculate_sum((unsigned char*)&conf, sizeof(conf));
  eeprom_write_block( (void*)&conf, (void*)(offset+sizeof(global_conf)), sizeof(conf) );
  //eeprom_write_block1( (void*)&conf, sizeof(global_conf), sizeof(conf) );
  readEEPROM(); //repeat to read eeprom after update conf
  if(b == 1) ledTwinkle(10); // writeParams complete 
}

void update_constants(void){
#if defined(GYRO_SMOOTHING)
  unsigned char s[3] = GYRO_SMOOTHING;
  for(unsigned char i = 0; i<3; i++)
    conf.Smoothing[i] = s[i];
#endif
  
  /* update others conf member*/
  
#if defined(FAILSAFE)
  conf.failsafe_throttle = FAILSAFE_THROTTLE; // maybe more to down slowly
#endif
  
  conf.minthrottle = MINTHROTTLE;
  
  writeParams(0);
}
