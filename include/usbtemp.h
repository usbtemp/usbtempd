#ifndef _ONEWIRE_DS18B20
#define _ONEWIRE_DS18B20

#define DS18X20_ROM_SIZE 8
#define DS18X20_SP_SIZE 9

char *DS18B20_errmsg(void);
int DS18B20_open(const char *);
int DS18B20_measure(int);
int DS18B20_acquire(int, float *);
int DS18B20_rom(int, unsigned char *);

#endif