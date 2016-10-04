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
#define FALSE 0
#define TRUE 1

using namespace std;

float accel[3];
float gyro[3];
float gyro_old = 0;
float angle[2];
float CFangle, CFangle_old = 0;
float pid_int =  0, pid_v = 0;
float pid_old = 0;
int pwm = 0;
float pterm, dterm, iterm;

/* ---- PID gain values ---- 
--------------------------*/
//float kp = 30, ki = 0, kd = 1.25, kv = 0;
float kp = 15, ki = 0, kd = 0.05, kv = 0;
/*------------------------*/

unsigned long time_old = 0;
unsigned long time_new = 0;
float DT;
//void signal_handler_IO (int status); //For UART READ
int wait_flag=TRUE;

// --- Function Prototypes ---
void get_accel(void);
void get_gyro(void);

int main(void) {
  struct timespec time_tag;
  struct termios oldtio, newtio;

  int msp_fs, res;
  
  ofstream fs ("data.txt"); //datalogging
  open_bus(); // I2C Bus

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

  // --- WRITE ACCELEROMETER CONTROL BYTES ---
  set_address(ACCEL_ADDRESS);
  buffer[0] = 0x10;
  buffer[1] = 0x80;
  write_i2c(buffer, 2);
  printf("Accel Control bytes written\n");

  // --- WRITE GYRO CONTROL BITS ---
  set_address(ACCEL_ADDRESS);
  buffer[0] = 0x11;
  buffer[1] = 0x80;  
  write_i2c(buffer, 2);
  printf("Gyro Control bytes written\n");

  int count = 1;
  
  while(1) {
    // --- READ ACCELEROMETER VALUES
    get_accel();
    get_gyro();
    clock_gettime(CLOCK_MONOTONIC, &time_tag);
    time_new = time_tag.tv_nsec;
    DT = (float) (time_new - time_old) / (1000000000.0);
    if(DT >= 1) DT = 0.0064;
    time_old = time_tag.tv_nsec;

    // --- CONVERSION FOR ANGLE APPROXIMATION ---
    angle[0] = -atan2(accel[1], sqrt(pow(-accel[2],2) + pow(-accel[0],2)));
    angle[1] = -atan2(-accel[2], sqrt(pow(accel[1],2) + pow(-accel[0],2)));

    // --- COMPLEMENTARY FILTER ---
    CFangle = (0.98 * (CFangle_old + gyro_old*(DT))) + (0.02 * ((angle[0]*180)/M_PI));

    // --- OUTPUT VALUES ---
    printf("Tilt: %-3.4f\tRoll: %-3.4f\t", (angle[0]*180.0)/M_PI, (angle[1]*180.0)/M_PI);
    printf("Gyro x: %-6f\ty:%6f\tz:%6f\t", gyro[0], gyro[1], gyro[2]);
    printf("Clock value: %lu\t", time_tag.tv_nsec);
    printf("dt: %-2.6f\t", DT);
    printf("CF Angle: %-3.4f\n", CFangle);

    
    if(fs.is_open())
      {
	fs << count << "\t";
	fs << (angle[0]*180.0)/M_PI << "\t";
	fs << (angle[1]*180.0)/M_PI << "\t";
	fs << gyro[0] << "\t" << gyro[1] << "\t" << gyro [2] << "\t";
	fs << DT << "\t";
	fs << CFangle << endl;
      }
    count++;
    

    /* --- Data Crunching Here --- */
    // Input will be CFangle - filtered tilt angle
    // Output to be motor direction and PWM values
    //pterm = kp*CFangle;
    //dterm = kd*(CFangle - CFangle_old);
    //iterm += ki*CFangle;

    pid_int += (CFangle)*DT; //change back to CFangle !!!!!!!!!
    pid_v += pid_old*DT;

    pwm = -(kp*(CFangle)) - (ki*pid_int) - (kd*gyro[2]) - (kv*pid_v); //change back to CFangle !!!!!!
    
    //pwm = -(pterm + iterm + dterm);
    if(pwm > 127) pwm = 127;
    if (pwm < -127) pwm = -127;
    cout << "PWM value: " << pwm << endl;

    /* --- UART TO MSP --- */
    unsigned char msp_data[2];
    
    //msp_data[0] = 0x7F; //start byte
    //msp_data[4] = 0x7E; //Stop Byte

    if(pwm > 0)
      {
	msp_data[0] = pwm; //first data byte
	msp_data[0] |= 0x80;
	//msp_data[3] = pwm; //second data byte
	//msp_data[3] |= 0x80;
      }
    else
      {
	msp_data[0] = -pwm;
	msp_data[0] &= ~0x80;
	//msp_data[3] = -pwm;
	//msp_data[3] &= ~0x80;
      }
    //msp_data[1] = msp_data[2] + msp_data[3]; //checksum
    msp_data[1] = 0; //null char to terminate char string
    

    res = write(msp_fs, msp_data, sizeof(msp_data) - 1);
    cout << res << " Bytes written to MSP" << endl;
    
    gyro_old = gyro[2];
    pid_old = pwm;
    CFangle_old = CFangle;
    usleep(1000);
  }
  //fs.close();
  tcsetattr(msp_fs, TCSANOW, &oldtio);
  close(msp_fs);
  return 1;
} //end main

void get_accel(void) {
  int Ax, Ay, Az;
  unsigned char out[6];

  set_address(0x6B);
  out[0] = 0x28;
  write_i2c(out, 1);
  read_i2c(out, 6);

  Ax = (short)(out[1] << 8 | out[0]);
  Ay = (short)(out[3] << 8 | out[2]);
  Az = (short)(out[5] << 8 | out[4]);

  accel[0] = (float)((2.0*Ax)/32767.0); //Ax
  accel[1] = (float)((2.0*Ay)/32767.0); //Ay
  accel[2] = (float)((2.0*Az)/32767.0); //Az
}

void get_gyro(void) {
  int Gx, Gy, Gz;
  unsigned char out[6];

  set_address(0x6B);
  out[0] = 0x22;
  write_i2c(out, 1);
  read_i2c(out, 6);

  Gx = (short)(out[1] << 8 | out[0]);
  Gy = (short)(out[3] << 8 | out[2]);
  Gz = (short)(out[5] << 8 | out[4]);

  gyro[0] = (float)((245.0*Gx)/32767.0); //Gx
  gyro[1] = (float)((245.0*Gy)/32767.0); //Gy
  gyro[2] = (float)((245.0*Gz)/32767.0); //Gz
}
