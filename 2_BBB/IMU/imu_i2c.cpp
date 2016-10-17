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

#define DATALOG_ON 1	// Comment out to disable data logging 
#define ALPHA 0.98
#define SMA_GYRO_EN 0
#define SMA_TILT_EN 0
#define SMA_PERIOD 1
#define SMA_PERIOD_TILT 1
#define ANGLE_OFFSET -6
#define PWM_LIMIT 2
#define DEADBAND_LIMIT 0.5

using namespace std;

float accel[3];
float gyro[3];
float gyro_old = 0;
float angle;
float CFangle, CFangle_old = 0;
float pid_int =  0, pid_v = 0;
float pid_old = 0;
int pwm, pwm_write;

/* ---- PID gain values ---- 
--------------------------*/
//float kp = 30, ki = 0, kd = 1.25, kv = 0;
float kp = 40, ki = 0.001, kd = 1, kv = 0;
/*------------------------*/

// SMA variables
float SMA_buffer[SMA_PERIOD+1], SMA_buf_tilt[SMA_PERIOD_TILT+1];
int SMA_buffer_startIdx = 0, SMA_buffer_endIdx = 0, sampleCount = SMA_PERIOD+1;
int SMA_buf_tilt_startIdx = 0, SMA_buf_tilt_endIdx = 0, sampleCount_tilt = SMA_PERIOD_TILT+1;
float SMA_sum = 0, gyro_z_SMA, SMA_sum_tilt = 0, tilt_SMA;

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
 
  #ifdef DATALOG_ON
  ofstream fs ("data.txt"); //datalogging
  #endif

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
  buffer[0] = 0x16;
  buffer[1] = 0x78;
  write_i2c(buffer, 2);
  printf("Gyro Control bytes written\n");

  int count = 1;
  static int n = SMA_PERIOD + 1;
  static int m = SMA_PERIOD_TILT + 1;
  
  while(1) {
    // --- READ ACCELEROMETER VALUES
    get_accel();
    get_gyro();
    clock_gettime(CLOCK_MONOTONIC, &time_tag);
    time_new = time_tag.tv_nsec;
    DT = (float) (time_new - time_old) / (1000000000.0);
    if(DT >= 1) DT = 0.003;
    time_old = time_tag.tv_nsec;

    // --- CONVERSION FOR ANGLE APPROXIMATION ---
    angle = -atan2(accel[1], sqrt(pow(-accel[2],2) + pow(-accel[0],2)));

    /*
    // --- SMA - ACCEL TILT ANGLE ---
    if(SMA_TILT_EN){
        if(sampleCount_tilt > 0) sampleCount_tilt--;
        SMA_buf_tilt[SMA_buf_tilt_endIdx] = angle;
        if((SMA_buf_tilt_startIdx <= SMA_buf_tilt_endIdx) && SMA_buf_tilt_endIdx != SMA_PERIOD_TILT){
            SMA_buf_tilt_endIdx = (SMA_buf_tilt_endIdx+1)%m;
        }else{
            SMA_buf_tilt_startIdx = (SMA_buf_tilt_startIdx+1)%m;
            SMA_buf_tilt_endIdx = (SMA_buf_tilt_endIdx+1)%m;
        }

        SMA_sum_tilt += angle;
        if(!sampleCount_tilt) SMA_sum_tilt -= SMA_buf_tilt[SMA_buf_tilt_startIdx];
        tilt_SMA = SMA_sum_tilt / SMA_PERIOD_TILT;
    }

    // --- SMA - GYRO Z ---
    if(SMA_GYRO_EN){
        if(sampleCount > 0) sampleCount--;
        SMA_buffer[SMA_buffer_endIdx] = gyro[2];
        if((SMA_buffer_startIdx <= SMA_buffer_endIdx) && SMA_buffer_endIdx != SMA_PERIOD){
            SMA_buffer_endIdx = (SMA_buffer_endIdx+1)%n;
        }else{
            SMA_buffer_startIdx = (SMA_buffer_startIdx+1)%n;
            SMA_buffer_endIdx = (SMA_buffer_endIdx+1)%n;
        }

        SMA_sum += gyro[2];
        if(!sampleCount) SMA_sum -= SMA_buffer[SMA_buffer_startIdx];
        gyro_z_SMA = SMA_sum / SMA_PERIOD;
    }
    */
    
    // --- COMPLEMENTARY FILTER ---
    //if(SMA_TILT_EN){
	// CF uses filtered tilt
        //CFangle = (ALPHA * (CFangle_old + gyro_old*(DT))) + ((1-ALPHA) * ((tilt_SMA*180)/M_PI));
    //}else{
	// CF doesn't  use filtered tilt (but does use filtered gyro (old) if(SMA_GYRO_EN))
        CFangle = (ALPHA * (CFangle_old + gyro_old*(DT))) + ((1-ALPHA) * ((angle*180)/M_PI));
	//}

    // --- OUTPUT VALUES ---
    /*
    printf("Tilt: %-3.4f\t", (angle[0]*180.0)/M_PI);
    printf("Gyro x: %-6f\ty:%6f\tz:%6f\t", gyro[0], gyro[1], gyro[2]);
    printf("Clock value: %lu\t", time_tag.tv_nsec);
    printf("dt: %-2.6f\t", DT);
    */
    printf("CF Angle: %-3.4f\n", CFangle+ANGLE_OFFSET);
    //printf("gyro_z_SMA: %6f\tsampleCount: %i\n", gyro_z_SMA, sampleCount);

    
    // --- DATA LOGGING ---
    #ifdef DATALOG_ON
        if(fs.is_open())
        {
	    fs << count << "\t";
	    fs << (angle*180.0)/M_PI << "\t";
	    fs << gyro[0] << "\t" << gyro[1] << "\t" << gyro [2] << "\t";
	    fs << DT << "\t";
	    fs << CFangle+ANGLE_OFFSET << "\t";
	    fs << CFangle << "\t\n";
        }
   	count++;
    #endif
   
    
    /* ------ PID control ------
     *   Input will be CFangle - filtered tilt angle
     *   Output to be motor direction and PWM values
     */

    // Add to integral terms
    pid_int += (CFangle+ANGLE_OFFSET)*DT; //change back to CFangle !!!!!!!!!
    pid_v += pid_old*DT;

    // Update PWM value if bot is outside deadband
    //if((CFangle+ANGLE_OFFSET) > DEADBAND_LIMIT || (CFangle+ANGLE_OFFSET) < -DEADBAND_LIMIT){
    // if(SMA_GYRO_EN){
            // Use gyro_z_SMA for d term
    //      pwm = -(kp*(CFangle+ANGLE_OFFSET)) - (ki*pid_int) - (kd*gyro_z_SMA) - (kv*pid_v); //change back to CFangle !!!!!!

	    // Print P, I, D
	    //cout << "P: " << -kp*(CFangle+ANGLE_OFFSET) << " I: " << -ki*pid_int << " D: " << -kd*gyro_z_SMA << " V: " << -(kv*pid_v) << endl;
    //  }else{
            // Use gyro[2] for d term
	    pwm = -(kp*(CFangle+ANGLE_OFFSET)) - (ki*pid_int) - (kd*gyro[2]) - (kv*pid_v);

	    // Print P, I, D
	    //cout << "P: " << -kp*(CFangle+ANGLE_OFFSET) << " I: " << -ki*pid_int << " D: " << -kd*gyro[2] << " V: " << -(kv*pid_v) << endl;
	    //} 	
	    //}
    
    // Adjust pwm for offset (PWM_LIMIT)
    if(pwm >= 0) {pwm_write = pwm + PWM_LIMIT;}       //pid_old = pwm - PWM_LIMIT; }
    else if(pwm < 0) {pwm_write = pwm - PWM_LIMIT;}   //pid_old = pwm + PWM_LIMIT; }
    // Adjust pwm for max/min
    if(pwm_write > 126) pwm_write = 126;
    if (pwm_write < -126) pwm_write = -126;

    // Print pwm
    cout << "PWM: " << pwm_write << " DT: " << DT << endl;

	
    /* --- UART TO MSP --- */
	// {Start, message, checksum}
	
    unsigned char msp_data[4];
	// Start byte
    msp_data[0] = START_BYTE;
	// First and second message bytes (L_CTRL, R_CTRL)
    if(pwm_write > 0){
		msp_data[1] = (127 - pwm_write); 			
		msp_data[1] &= ~0x80;						// Clear PWM_DIR bit
		
		msp_data[2] = (127 - pwm_write); 			
		msp_data[2] &= ~0x80;						// Clear PWM_DIR bit
    }else{
		msp_data[1] = -(126 - pwm_write);
		msp_data[1] |= 0x80;						// Set PWM_DIR bit
		
		msp_data[2] = -(126 - pwm_write);
		msp_data[2] |= 0x80;						// Set PWM_DIR bit
    }

	// Checksum byte
    msp_data[3] = ~(msp_data[1] + msp_data[2]);
	
    //msp_data[1] = 0; //null char to terminate char string				// Null character needed?
    
    res = write(msp_fs, msp_data, sizeof(msp_data) - 1);				// sizeof(msp_data) - 1 ??
    //cout << res << " Bytes written to MSP" << endl;
    
    // Update 'old' values for next loop
    //if(SMA_GYRO_EN){
    //    gyro_old = gyro_z_SMA;
    //}else{
	gyro_old = gyro[2];
	//}
    pid_old = pwm;
    CFangle_old = CFangle;
   // usleep(15000);
  }
  /*
  #ifdef DATALOG_ON
  fs.close();
  #endif
  */
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
