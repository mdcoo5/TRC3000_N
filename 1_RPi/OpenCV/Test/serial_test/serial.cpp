#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#define BAUDRATE B9600
#define DEVICE "/dev/ttyO1"
#define FALSE 0
#define TRUE 1

using namespace std;

int main(void)
{
  int fd;
  struct termios oldtio, newtio;

  //open modem device
  fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY );
  if (fd < 0) { cout << "Error opening device" << endl; return -1; }
  else cout << "Device opened successfully" << endl;

  tcgetattr(fd, &oldtio); // save current serial port settings
  bzero(&newtio, sizeof(newtio)); // clear struct to recieve new settings

  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR | ICRNL;
  newtio.c_oflag = 0;
  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG );

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  unsigned char buf[] = "Hello from a C++ code\r";
  unsigned char buf2[] = "Testing this feature ...\r";

  int num = 0;
  num = write(fd,buf,sizeof(buf)-1);
  cout << "Data Sent: " << num << " bytes" << endl;
  num = write(fd,buf2,sizeof(buf2)-1);
  cout << "Data Sent: " << num << " bytes" << endl;
  tcsetattr(fd, TCSANOW, &oldtio);
  close(fd);
  return 0;
}
  
