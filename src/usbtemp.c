#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "usbtemp.h"

#define TIMEOUT 1
#define DS18X20_GENERATOR 0x8c

static char* ut_msgs[] = {
  "",
  "Error, could not get baudrate!",
  "Error, could not set baudrate!",
  "Error, serial port does not exist!",
  "Error, you don't have rw permission to access serial port!",
  "Error, failed to open serial port device!",
  "Error, sensor not found!", /* 6 */
  "Error, sensor CRC mismatch!",
  "Warining, not expected sensor response!",
  "Error, could not send data!"
};

int ut_errno;

char *DS18B20_errmsg(void)
{
  return ut_msgs[ut_errno];
}

static unsigned char lsb_crc8(unsigned char *data_in, unsigned int len, const unsigned char generator)
{
  unsigned char i, bit_counter;
  unsigned char crc = 0;

  for (i = 0; i < len; i++) {
    crc ^= *(data_in + i);
    bit_counter = 8;
    do {
      if (crc & 0x01) {
        crc = (((crc >> 1) & 0x7f) ^ generator);
      }
      else {
        crc = (crc >> 1) & 0x7f;
      }
      bit_counter--;
    } while (bit_counter > 0);
  }
  return crc;
}

static int owReset(int fd)
{
  int rv;
  int wbytes, rbytes;
  unsigned char wbuff, rbuff;
  fd_set readset;
  struct timeval timeout_tv;
  struct termios term;

  tcflush(fd, TCIOFLUSH);

  if (tcgetattr(fd, &term) < 0) {
    ut_errno = 1;
    return -1;
  }
  term.c_cflag &= ~CSIZE;
  term.c_cflag |= CS8;
  cfsetispeed(&term, B9600);
  cfsetospeed(&term, B9600);
  tcsetattr(fd, TCSANOW, &term);

  /* Send the reset pulse. */
  wbuff = 0xf0;
  wbytes = write(fd, &wbuff, 1);
  if (wbytes != 1) {
    ut_errno = 9;
    return -1;
  }

  timeout_tv.tv_usec = 0;
  timeout_tv.tv_sec = TIMEOUT;

  FD_ZERO(&readset);
  FD_SET(fd, &readset);

  if (select(fd + 1, &readset, NULL, NULL, &timeout_tv) > 0) {

    if (FD_ISSET(fd, &readset)) {
      rbytes = read(fd, &rbuff, 1);
      if (rbytes != 1) {
        return -1;
      }
      switch (rbuff) {
        case 0:
          /* Ground. */
        case 0xf0:
          /* No response. */
          rv = -1;
          break;
        default:
          /* Got a response */
          rv = 0;
      }
    }
    else {
      rv = -1;
    }
  }
  else {
    rv = -1; /* Timed out or interrupt. */
  }

  term.c_cflag &= ~CSIZE;
  term.c_cflag |= CS6;
  cfsetispeed(&term, B115200);
  cfsetospeed(&term, B115200);

  tcsetattr(fd, TCSANOW, &term);

  return rv;
}

static unsigned char owWriteByte(int fd, unsigned char wbuff)
{
  char buf[8];
  int wbytes;
  unsigned char rbuff, i;
  size_t remaining, rbytes;
  fd_set readset;
  struct timeval timeout_tv;

  tcflush(fd, TCIOFLUSH);

  for (i = 0; i < 8; i++) {
    buf[i] = (wbuff & (1 << (i & 0x7))) ? 0xff : 0x00;
  }
  wbytes = write(fd, buf, 8);
  if (wbytes != 8) {
    ut_errno = 9;
    return -1;
  }

  timeout_tv.tv_usec = 0;
  timeout_tv.tv_sec = TIMEOUT;

  FD_ZERO(&readset);
  FD_SET(fd, &readset);

  rbuff = 0;
  remaining = 8;
  while (remaining > 0) {

    if (select(fd + 1, &readset, NULL, NULL, &timeout_tv) > 0) {

      if (FD_ISSET(fd, &readset)) {
        rbytes = read(fd, &buf, remaining);
        for (i = 0; i < rbytes; i++) {
          rbuff >>= 1;
          rbuff |= (buf[i] & 0x01) ? 0x80 : 0x00;
          remaining--;
        }
      }
      else {
        return 0xff;
      }
    }
    else {
      return 0xff;
    }
  }
  return rbuff;
}

static unsigned char owRead(int fd)
{
  return owWriteByte(fd, 0xff);
}

static int owWrite(int fd, unsigned char wbuff)
{
  return (owWriteByte(fd, wbuff) == wbuff) ? 0 : -1;
}

static int file_exists(const char *filename)
{
  struct stat st;

  return (stat(filename, &st) == 0);
}

static int DS18B20_start(int fd)
{
  if (owReset(fd) < 0) {
    ut_errno = 6;
    return -1;
  }
  if (owWrite(fd, 0xcc) < 0) {
    ut_errno = 8;
    return -1;
  }
  return 0;
}

int DS18B20_measure(int fd)
{
  if (DS18B20_start(fd) < 0) {
    return -1;
  }
  if (owWrite(fd, 0x44) < 0) {
    ut_errno = 8;
    return -1;
  }
  return 0;
}

static int DS18B20_sp(int fd, unsigned char *sp)
{
  unsigned char i, crc;

  if (DS18B20_start(fd) < 0) {
    return -1;
  }
  if (owWrite(fd, 0xbe) < 0) {
    ut_errno = 8;
    return -1;
  }
  for (i = 0; i < DS18X20_SP_SIZE; i++) {
    *(sp + i) = owRead(fd);
  }

  if ((*(sp + 4) & 0x9f) != 0x1f) {
    ut_errno = 6;
    return -1;
  }

  crc = lsb_crc8(sp, DS18X20_SP_SIZE - 1, DS18X20_GENERATOR);
  if (*(sp + DS18X20_SP_SIZE - 1) != crc) {
    ut_errno = 7;
    return -1;
  }

  return 0;
}

int DS18B20_setprecision(int fd, int precision)
{
  int i, rv;
  unsigned char cfg_old, cfg[4], sp_sensor[DS18X20_SP_SIZE];
  unsigned char *p;

  p = cfg + 3;
  *p = 0x1f | ((unsigned char)(precision - 9) << 5);

  rv = DS18B20_sp(fd, sp_sensor);
  if (rv < 0) {
    return rv;
  }

  cfg_old = sp_sensor[DS18B20_SP_CONFIG];
  if (cfg_old == *p) {
    return 0;
  }

  p--;
  *p-- = sp_sensor[DS18B20_SP_TL];
  *p-- = sp_sensor[DS18B20_SP_TH];
  *p = DS18B20_SP_WRITE;

  if (DS18B20_start(fd) < 0) {
    return -1;
  }
  for (i = 0; i < 4; i++) {
    if (owWrite(fd, *p++) < 0) {
      ut_errno = 8;
      return -1;
    }
  }

  if (DS18B20_start(fd) < 0) {
    return -1;
  }
  if (owWrite(fd, DS18B20_SP_SAVE) < 0) {
    ut_errno = 8;
    return -1;
  }

  return 0;
}

int DS18B20_acquire(int fd, float *temperature)
{
  int rv;
  short T;
  unsigned short uT;
  unsigned char sp_sensor[DS18X20_SP_SIZE];

  rv = DS18B20_sp(fd, sp_sensor);
  if (rv < 0) {
    return rv;
  }

  uT = (sp_sensor[1] << 8) | (sp_sensor[0] & 0xff);
  T = (short)uT; /* Force using leading bit as a sign. */
  *temperature = (float)T / 16;

  return 0;
}

int DS18B20_rom(int fd, unsigned char *rom)
{
  unsigned char i, crc;

  if (owReset(fd) < 0) {
    ut_errno = 6;
    return -1;
  }
  if (owWrite(fd, 0x33) < 0) {
    ut_errno = 8;
    return -1;
  }

  for (i = 0; i < DS18X20_ROM_SIZE; i++) {
    rom[i] = owRead(fd);
  }

  crc = lsb_crc8(rom, DS18X20_ROM_SIZE - 1, DS18X20_GENERATOR);
  if (*(rom + DS18X20_ROM_SIZE - 1) != crc) {
    ut_errno = 7;
    return -1;
  }

  return 0;
}

int is_fd_valid(int fd)
{
  return (fd > 0);
}

int owOpen(const char *serial_port)
{
  int fd;
  struct termios term;

  if (!file_exists(serial_port)) {
    ut_errno = 3;
    return -1;
  }

  if (access(serial_port, R_OK|W_OK) < 0) {
    ut_errno = 4;
    return -1;
  }

  fd = open(serial_port, O_RDWR);
  if (fd < 0) {
    ut_errno = 5;
    return -1;
  }

  memset(&term, 0, sizeof(term));

  term.c_cc[VMIN] = 1;
  term.c_cc[VTIME] = 0;
  term.c_cflag |= CS6 | CREAD | HUPCL | CLOCAL;

  cfsetispeed(&term, B115200);
  cfsetospeed(&term, B115200);

  if (tcsetattr(fd, TCSANOW, &term) < 0) {
    close(fd);
    ut_errno = 2;
    return -1;
  }

  tcflush(fd, TCIOFLUSH);

  return fd;
}

int DS18B20_open(const char *serial_port)
{
  return owOpen(serial_port);
}

static void owClose(int fd)
{
  close(fd);
}

void DS18B20_close(int fd)
{
  return owClose(fd);
}

void wait_1s(void)
{
  struct timeval wait_tv;

  wait_tv.tv_usec = 0;
  wait_tv.tv_sec = 1;
  select(0, NULL, NULL, NULL, &wait_tv);
}