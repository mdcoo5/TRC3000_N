#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#define BAUDRATE B9600
#define DEVICE "/dev/rfcomm0"
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
  newtio.c_lflag = ICANON;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);
  
  unsigned char type; //F - 0x01, R - 0x02, L - 0x04, Right - 0x08, STOP - 0x10
  unsigned char len;
  unsigned char chksum;
  unsigned char data[4];

  //message construction
  type = 0x01;
  data[0] = 50;
  data[1] = 50;
  data[2] = 0;
  data[3] = 0;
  len = 2;
  chksum = data[0] + data [1] + data[2] + data[3];
  chksum = ~chksum;

  unsigned char buf[] = {0x7F, type, len, chksum, data[0], data[1], data[2], data[3], 0}; // {START, TYPE, LEN, CHKSUM, DATA, NULL}

  //output data
  printf("ST: %x, TY: %x, LN: %x, CHK: %x, DT: %x, %x, %x, %x\n", 0x7F, type, len, chksum, data[0], data[1], data[2], data[3]);
  
  int num = 0;
  num = write(fd,buf,sizeof(buf)-1);
  cout << "Data Sent: " << num << " bytes" << endl;
  tcsetattr(fd, TCSANOW, &oldtio);
  close(fd);
  
  return 0;
}
