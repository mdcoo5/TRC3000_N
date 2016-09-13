#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/signal.h>

#define BAUDRATE B115200
#define DEVICE "/dev/ttyO1"
#define FALSE 0
#define TRUE 1

using namespace std;
volatile int STOP=FALSE;
void signal_handler_IO (int status);
int wait_flag=TRUE;

int main(void)
{
  int fd;
  struct termios oldtio, newtio;
  struct sigaction saio;

  //open modem device
  fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY );
  if (fd < 0) { cout << "Error opening device" << endl; return -1; }
  else cout << "Device opened successfully" << endl;

  saio.sa_handler = signal_handler_IO;
  saio.sa_mask = 0;
  saio.sa_flags = 0;
  saio.sa_restorer = NULL;
  sigaction(SIGIO,&saio,NULL);

  fcntl(fd, F_SETOWN, getpid());
  fcntl(fd, F_SETFL, FASYNC);

  tcgetattr(fd, &oldtio); // save current serial port settings
  bzero(&newtio, sizeof(newtio)); // clear struct to recieve new settings

  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR | ICRNL;
  newtio.c_oflag = 0;
  newtio.c_lflag = ICANON;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  int res = 0;
  char buf[255];

  while(STOP==FALSE)
    {
      usleep(10000);
      if(wait_flag==FALSE)
	{
	  int res = read(fd, buf, 255);
	  buf[res] = 0;
	  cout << buf << res << endl;
	  if (res == 1) STOP=TRUE;
	  wait_flag = TRUE;
	}
    }

  tcsetattr(fd, TCSANOW, &oldtio);
  close(fd);
  return 0;
}
  
