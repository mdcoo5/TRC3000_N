#include <stdio.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <signal.h>
#include "i2c_function_decs.h"

#define ACCEL_ADDRESS 0x6B
#define MAG_ADDRESS 0x1E
#define MSP_BAUDRATE B115200
#define MSP_DEVICE "/dev/ttyO2"
#define START_BYTE 0x7F
#define FALSE 0
#define TRUE 1

// #define DATALOG_ON 1	// Comment out to disable data logging 

using namespace std;

int pwm_write = 50;
unsigned long time_old = 0;
unsigned long time_new = 0;
float DT;
//void signal_handler_IO (int status); //For UART READ
int wait_flag=TRUE;

int main(void) {
  struct timespec time_tag;
  struct termios oldtio, newtio;

  int msp_fs, res;
  int state = 0;
 
  #ifdef DATALOG_ON
  ofstream fs ("data.txt"); //datalogging
  #endif

  //open_bus(); // I2C Bus

  // open serial port
  msp_fs = open(MSP_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
  if (msp_fs < 0) { cout << "Error opening MSP Serial port" << endl; return -1; }
  else cout << "MSP device opened successfully" << endl;

  //Configure Serial port
  //fcntl(msp_fs, F_SETOWN, getpid());
  //fcntl(msp_fs, F_SETFL, FASYNC);
  tcgetattr(msp_fs, &oldtio); // save current serial port settings
  bzero(&newtio, sizeof(newtio)); //clear struct for new settings

  newtio.c_cflag = MSP_BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR | ICRNL;
  newtio.c_oflag = 0;
  newtio.c_lflag = ICANON;

  // Set serial port to new settings
  tcflush(msp_fs, TCIFLUSH);
  tcsetattr(msp_fs, TCSANOW, &newtio);
  
  while(1) {
    clock_gettime(CLOCK_MONOTONIC, &time_tag);
    time_new = time_tag.tv_nsec;
    DT = (float) (time_new - time_old) / (1000000000.0);
    if(DT >= 1) DT = 0.003;
    time_old = time_tag.tv_nsec;

    // --- OUTPUT VALUES ---
    /*
    printf("dt: %-2.6f\t", DT);
    */

    /* 
    // --- DATA LOGGING ---
    #ifdef DATALOG_ON
        if(fs.is_open())
        {
	    fs << DT << "\t\n";
        }
    #endif
    */
    
    
    /* --------------- UART TO MSP --------------- */
	// {Start, message, checksum}
	
    unsigned char msp_data[4];
	// Start byte
    msp_data[0] = START_BYTE;
	
	
	// Test code (First and second message bytes (L_CTRL, R_CTRL))
	switch(state){
		case 0:									// Forward case
			msp_data[1] = (126-pwm_write);
			msp_data[1] &= ~0x80;				// Clear PWM_DIR bit
			msp_data[2] = (126 - pwm_write); 			
			msp_data[2] &= ~0x80;				// Clear PWM_DIR bit
			state++;
			break;
		case 1:
			msp_data[1] = -(126 - pwm_write);
			msp_data[1] |= 0x80;						// Set PWM_DIR bit
			msp_data[2] = -(126 - pwm_write);
			msp_data[2] |= 0x80;						// Set PWM_DIR bit
			state = 0;
			break;
	}
    // Checksum byte
    msp_data[3] = ~(msp_data[1] + msp_data[2]);
    
    res = write(msp_fs, msp_data, sizeof(msp_data));			
    /* -------------------------------------------- */
	
    usleep(1000000);
  }
  
  #ifdef DATALOG_ON
  fs.close();
  #endif
  
  tcsetattr(msp_fs, TCSANOW, &oldtio);
  close(msp_fs);
  return 1;
} //end main
