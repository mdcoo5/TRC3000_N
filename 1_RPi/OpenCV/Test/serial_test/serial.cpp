#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#define BAUDRATE B115200
#define DEVICE "/dev/rfcomm0"
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;

int main(void)
{
  int fd,  res;
  struct termios oldtio, newtio;
  char buf[255];
  
  // Open file descriptor
  fd = open(DEVICE, O_RDWR | O_NOCTTY );
  // Error Handling
  if(fd < 0){ perror(DEVICE); return -1;}

  tcgetattr(fd, &oldtio); // save current serial port settings
  bzero(&newtio, sizeof(newtio)); // clears struct for new port settings

  // setup new port
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

  newtio.c_iflag = IGNPAR | ICRNL;

  newtio.c_oflag = 0;

  newtio.c_lflag = ICANON;

  // cleanse modem line
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  // input handling
  while (STOP==FALSE) {
    res = read(fd, buf, 255);
    buf[res] = 0;
    printf(":%s:%d\n", buf, res);
    if(buf[0]=='z') STOP=TRUE;
  }
  tcsetattr(fd,TCSANOW,&oldtio);
  return 0;
}
  
