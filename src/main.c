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
  int enable = 1;

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
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
      printf("setsockopt(SO_REUSEADDR) failed");
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
    return -1;
  }

  return sockfd;
}

static int valid = 0;
static unsigned char rom[DS18X20_ROM_SIZE];
static float temp;

static int verbose;
static int onewire_pipe[2], net_pipe[2];

void terminate(void)
{
  uint8_t c = 0;
  write(net_pipe[1], &c, 1);
  write(onewire_pipe[1], &c, 1);
}

void *onewire(void *arg)
{
  int serial_fd;
  char *serialport;
  fd_set readfds;
  struct timeval wait_tv;

  serial_fd = 0; /* disconnected: 0 */

  while (1) {

    if (!serial_fd) {
      valid = 0;
      serialport = (char *)arg;

      if (serial_fd > 0) {
        close(serial_fd);
        serial_fd = 0;
      }
      serial_fd = DS18B20_open(serialport);

      if (serial_fd < 0) {
        fprintf(stderr, "Cannot open serial port!\n");
        serial_fd = 0;

        wait_tv.tv_sec = 5;
        wait_tv.tv_usec = 0;
        FD_ZERO(&readfds);
        FD_SET(onewire_pipe[0], &readfds);
        if (select(onewire_pipe[0] + 1, &readfds, NULL, NULL, &wait_tv)) {
          break;
        }
        continue;
      }
      else if (DS18B20_rom(serial_fd, rom) < 0) {
        fprintf(stderr, "%s\n", DS18B20_errmsg());
        close(serial_fd);
        serial_fd = 0;
        continue;
      }
      else {
        printf("Connected to %02x%02x%02x%02x%02x%02x%02x%02x.\n", *rom, *(rom + 1), *(rom + 2), *(rom + 3), *(rom + 4), *(rom + 5), *(rom + 6), *(rom + 7));
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
    FD_ZERO(&readfds);
    FD_SET(onewire_pipe[0], &readfds);
    if (select(onewire_pipe[0] + 1, &readfds, NULL, NULL, &wait_tv)) {
      break;
    }

    if (DS18B20_acquire(serial_fd, &temp) < 0) {
      fprintf(stderr, "%s\n", DS18B20_errmsg());
      close(serial_fd);
      serial_fd = 0;
      continue;
    }
    valid = 1;
  }

  if (serial_fd > 0) {
    close(serial_fd);
    serial_fd = 0;
  }

  if (verbose) {
    printf("onewire thread quit\n");
  }

  terminate();

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
  int max_fd;
  char *name;

  listen_fd = create_listen_socket();
  if (listen_fd < 0) {
    goto end;
  }

  name = (char *)arg;
  while (1) {

    FD_ZERO(&readfds);
    FD_SET(listen_fd, &readfds);
    FD_SET(net_pipe[0], &readfds);
    max_fd = listen_fd > net_pipe[0] ? listen_fd : net_pipe[0];
    select_rv = select(max_fd + 1, &readfds, NULL, NULL, NULL);
    if (FD_ISSET(net_pipe[0], &readfds)) {
      break;
    }

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

  if (verbose) {
    printf("net thread quit\n");
  }

end:

  terminate();

  return NULL;
}

void signal_handler(int signum)
{
  if (signum == SIGTERM) {
    if (verbose) {
      printf("SIGTERM received, shutting down ...\n");
    }
    signal(SIGTERM, SIG_IGN);
    terminate();
  }
  if (signum == SIGINT) {
    if (verbose) {
      printf("SIGINT received, shutting down ...\n");
    }
    signal(SIGINT, SIG_IGN);
    terminate();
  }
}

int main(int argc, char **argv)
{
  char c;
  char *serialport = NULL;
  char *name = NULL;

  int iret1, iret2;
  pthread_t net_thread, onewire_thread;
  struct sigaction sa;

  sa.sa_flags = 0;
  sa.sa_handler = signal_handler;
  sigemptyset(&sa.sa_mask);
  if (sigaction(SIGTERM, &sa, NULL) < 0 || sigaction(SIGINT, &sa, NULL) < 0 ) {
    fprintf(stderr, "Signal handler could not be set!\n");
    return 1;
  }

  verbose = 0;

  while ((c = getopt(argc, argv, "n:s:v")) != -1) {
    switch (c) {

      case 'n':
        if (name) {
          free(name);
        }
        name = strdup(optarg);
        break;

      case 's':
        if (serialport) {
          free(serialport);
        }
        serialport = strdup(optarg);
        break;

      case 'v':
        verbose = 1;
        break;
    }
  }
  if (!serialport) {
    serialport = strdup("/dev/ttyUSB0");
    if (verbose) {
      printf("Serial port undefined, using %s as a serial port.\n", serialport);
    }
  }

  pipe(net_pipe);
  pipe(onewire_pipe);

  iret1 = pthread_create(&net_thread, NULL, net, (void *)name);
  iret2 = pthread_create(&onewire_thread, NULL, onewire, (void *)serialport);

  if (!iret1 && !iret2) {
    if (verbose) {
      printf("Threads are ready.\n");
    }
    pthread_join(net_thread, NULL);
    pthread_join(onewire_thread, NULL);
  }
  else {
    terminate();
  }

  free(serialport);

  return 0;
}