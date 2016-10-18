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
#define START_BYTE 0x7F

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
  unsigned char data[9];
  unsigned char types[5] = {0x01, 0x02, 0x04, 0x08, 0xFF};
    
  int num = 0;
  int i = 0;
  
  
  data[0] = START_BYTE; // START BYTE
  data[1] = 1; // MSG_LENGTH
  
  while(1){
    // TYPE BIT
  	data[3] = types[i];
  	// DATA BITS
  	data[4] = 0x00;
  	data[5] = 0x00;
  	data[6] = 0x00;
  	data[7] = 0x00;
  	
  	data[2] = ~(data[3] + data[4] + data[5] + data[6] + data[7]);

	data[8] = '\r';
	
  	num = write(fd,data,sizeof(data));
	printf("TYPE: %x , CS: %x ", data[3], data[2]);
  	cout << "Data Sent: " << num << " bytes" << endl;
  	i++;
  	if(i>4) i = 0;
  	usleep(1000000);
  }
  tcsetattr(fd, TCSANOW, &oldtio);
  close(fd);
  return 0;
}
