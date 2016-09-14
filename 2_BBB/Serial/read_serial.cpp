#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <signal.h>

#define BAUDRATE B9600
#define DEVICE "/dev/ttyO1"
#define DEVICE_W "/dev/ttyO2"
#define FALSE 0
#define TRUE 1

using namespace std;
volatile int STOP=FALSE;
void signal_handler_IO (int status);
int wait_flag=TRUE;

int main(void)
{
  int fd, fw, res;
  struct termios oldtio, oldtio_w, newtio;
  struct sigaction saio;
  unsigned char buf[255];

  //open modem device
  fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY );
  if (fd < 0) { cout << "Error opening device" << endl; return -1; }
  else cout << "Device opened successfully" << endl;

  saio.sa_handler = signal_handler_IO;
  sigemptyset(&saio.sa_mask);
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

  fw = open(DEVICE_W, O_RDWR | O_NOCTTY | O_NDELAY );
  if (fw < 0) { cout << "Error opening write device" << endl; return -1; }
  else cout << "Write device opened succesfully" << endl;

  tcgetattr(fw, &oldtio_w);
  tcflush(fw, TCIFLUSH);
  tcsetattr(fw, TCSANOW, &newtio);

  while(STOP==FALSE)
    {
      usleep(1000);
      //printf(".\n");
      if(wait_flag==FALSE)
	{
	  res = read(fd, buf, 255);
	  buf[res] = 0;
	  printf("%s", buf);
	  unsigned char out[2] = {buf[0], '\r'};
	  int writ = write(fw,out,sizeof(out)-1);
	  cout << "Data sent: " << out[0] << " : "<< writ << " bytes" << endl;
	  if (res == 1) STOP=TRUE;
	  wait_flag = TRUE;
	}
    }

  tcsetattr(fd, TCSANOW, &oldtio);
  tcsetattr(fw, TCSANOW, &oldtio_w);
  close(fd);
  close(fw);
  return 0;
}

void signal_handler_IO (int status)
{
  //std::cout << "recieved SIGIO Signal" << std::endl;
  wait_flag=FALSE;
}
