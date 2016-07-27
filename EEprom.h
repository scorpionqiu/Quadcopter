#ifndef EEPROM_H_
#define EEPROM_H_
void readGlobalSet(void);
bool readEEPROM(void);
void update_constants(void);
void writeGlobalSet(unsigned char b);
void writeParams(unsigned char b);
void LoadDefaults(void);
void readPLog(void);
void writePLog(void);
void eeprom_read_block(void*, void*, unsigned char);
//void eeprom_write_block(void*, void*, unsigned char);

unsigned char eeprom_read_byte(unsigned int add);
void eeprom_write_byte(unsigned int add, unsigned char data);
#endif