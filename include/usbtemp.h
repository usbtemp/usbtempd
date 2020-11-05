#ifndef _ONEWIRE_DS18B20
#define _ONEWIRE_DS18B20

#define DS18X20_GENERATOR 0x8c
#define DS18X20_ROM_SIZE 8
#define DS18X20_SP_SIZE 9
#define DS18B20_SP_TH 2
#define DS18B20_SP_TL 3
#define DS18B20_SP_CONFIG 4

#define DS18B20_SP_WRITE 0x4e
#define DS18B20_SP_SAVE 0x48

char *DS18B20_errmsg(void);
int DS18B20_open(const char *);
int DS18B20_measure(int);
int DS18B20_acquire(int, float *);
int DS18B20_rom(int, unsigned char *);

#endif