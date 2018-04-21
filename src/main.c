#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include "usbtemp.h"

#define BACKLOG 10
#define PORT 2000
 
int create_listen_socket()
{
  struct addrinfo hints, *servinfo, *ptr;
  int rv;
  int sockfd;
  char port[6];
  
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_flags = AI_PASSIVE;
  hints.ai_socktype = SOCK_STREAM;
  
  sprintf(port, "%d", PORT);
  
  rv = getaddrinfo(NULL, port, &hints, &servinfo);
  if (rv) {
    printf("getaddrinfo() error\n");
    return -1;
  }
  
  for (ptr = servinfo; ptr; ptr = ptr->ai_next) {
    sockfd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol);
    if (sockfd < 0) {
      continue;
    }
    rv = bind(sockfd, servinfo->ai_addr, servinfo->ai_addrlen);
    if (rv < 0) {
      close(sockfd);
      fprintf(stderr, "Port %s already in use!\n", port);
      continue;
    }
    listen(sockfd, BACKLOG);
    rv = fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL) | O_NONBLOCK);
    if (rv < 0) {
      close(sockfd);
      printf("fcntl() error\n");
    }
    break;
  }
  if (!ptr) {
    sockfd = -1;
  }

  return sockfd;
}

static volatile int failure = 0;
static volatile int valid = 0;
static unsigned char rom[DS18X20_ROM_SIZE];
static float temp;

void *onewire(void *arg)
{
  int serial_fd;
  char *serialport;
  struct timeval wait_tv;
  
  serial_fd = 0;
  
  while (!failure) {

    if (!serial_fd)
    {
      valid = 0;
      serialport = (char *)arg;
      if (serial_fd > 0)
      {
        close(serial_fd);
      }
      serial_fd = DS18B20_open(serialport);
      if (serial_fd < 0)
      {
        serial_fd = 0;
        wait_tv.tv_sec = 5;
        wait_tv.tv_usec = 0;
        select(0, NULL, NULL, NULL, &wait_tv);
        continue;
      }
      else if (DS18B20_rom(serial_fd, rom) < 0)
      {
        fprintf(stderr, "%s\n", DS18B20_errmsg());
        close(serial_fd);
        serial_fd = 0;
        continue;
      }
      else
      {
        printf("Connected.\n");
      }
    }
    
    if (DS18B20_measure(serial_fd) < 0) {
      fprintf(stderr, "%s\n", DS18B20_errmsg());
      close(serial_fd);
      serial_fd = 0;
      continue;
    }

    wait_tv.tv_usec = 0;
    wait_tv.tv_sec = 1;
    select(0, NULL, NULL, NULL, &wait_tv);
    
    if (DS18B20_acquire(serial_fd, &temp) < 0) {
      fprintf(stderr, "%s\n", DS18B20_errmsg());
      close(serial_fd);
      serial_fd = 0;
      continue;
    }
    valid = 1;
  }

  close(serial_fd);

  return NULL;
}

void *net(void *arg)
{
  int listen_fd;
  int select_rv;
  int new_fd;
  struct sockaddr_storage their_addr;
  socklen_t sin_size;
  fd_set readfds;
  FILE *file;
  char *name;
  
  listen_fd = create_listen_socket();
  if (listen_fd < 0) {
    failure = 1;
    return NULL;
  }
  
  name = (char *)arg;
  while (!failure) {
    
    FD_ZERO(&readfds);
    FD_SET(listen_fd, &readfds);

    select_rv = select(listen_fd + 1, &readfds, NULL, NULL, NULL);

    if (select_rv != 1) {
      fprintf(stderr, "select() err!\n");
    }
    if (FD_ISSET(listen_fd, &readfds)) {
      sin_size = sizeof(struct sockaddr_storage);
      new_fd = accept(listen_fd, (struct sockaddr *)&their_addr, &sin_size);
      file = fdopen(new_fd, "w");
      if (name) {
        fprintf(file, "%s: ", name);
      }
      else {
        fprintf(file, "%02x%02x%02x%02x%02x%02x%02x%02x: ", *rom, *(rom + 1), *(rom + 2), *(rom + 3), *(rom + 4), *(rom + 5), *(rom + 6), *(rom + 7));
      }
      if (valid) {
        fprintf(file, "%.2f\n", temp);
      }
      else {
        fprintf(file, "-\n");
      }
      fflush(file);
      fclose(file);
      close(new_fd);
    }
  }
  close(listen_fd);
  
  return NULL;
}

void signal_handler(int signum)
{
  if (signum == SIGTERM) {
    failure = 1;
    signal(SIGTERM, SIG_IGN);
  }
}

int main(int argc, char **argv)
{
  char c;
  char *serialport = NULL;
  char *name = NULL;
  int iret1, iret2;
  pthread_t thread1, thread2;
  struct sigaction sa;
  
  sa.sa_flags = 0;
  sa.sa_handler = signal_handler;
  sigemptyset(&sa.sa_mask);
  if (sigaction(SIGTERM, &sa, NULL) < 0) {
    fprintf(stderr, "Signal handler could not be set!\n");
    return 1;
  }
  
  while ((c = getopt(argc, argv, "n:s:")) != -1) {
    switch (c) {

      case 'n':
        name = strdup(optarg);
        break;

      case 's':
        serialport = strdup(optarg);
        break;
    }
  }
  if (!serialport) {
    serialport = strdup("/dev/ttyUSB0");
  }
  
  iret1 = pthread_create(&thread1, NULL, net, (void *)name);
  iret2 = pthread_create(&thread2, NULL, onewire, (void *)serialport);
  
  if (!iret1 && !iret2) {
    printf("%s: Threads are ready.\n", argv[0]);
  }
  
  pthread_join(thread1, NULL);
  pthread_join(thread2, NULL);
  
  free(serialport);
  
  return 0;
}
