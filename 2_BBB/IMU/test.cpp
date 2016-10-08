/* --- INCLUDES ---- */
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

/* --- DEFINES --- */
#define ACCEL_ADDRESS 0x6B
#define MAG_ADDRESS 0x1E
#define MSP_BAUDRATE B115200
#define MSP_DEVICE "/dev/ttyO2"

#define FALSE 0
#define TRUE 1

#define DATALOG_ON 1	// Comment out to disable data logging 
#define ALPHA 0.98

#define ACCEL_SMA_ON 1
#define ACCEL_SMA_PERIOD 10
#define GYRO_SMA_ON 1
#define GYRO_SMA_PERIOD 10

#define ANGLE_OFFSET -5.5
#define PWM_MINIMUM 0
#define DEADBAND 0.5

using namespace std;

/* --- GLOBALS --- */
int accel_sma_buf[3][ACCEL_SMA_PERIOD];  	// [X, Y, Z]
int gyro_sma_buf[GYRO_SMA_PERIOD];   	// [X, Y, Z]
int accel[3], gyro;
long accel_tot[3];
long gyro_tot;
float accel_val[3], gyro_val;
float tilt_angle, cf;
float gyro_old, cf_old;
float pid_p, pid_i, pid_d, pid_v;
float DT;
int pwm = 0;

/* --- PID GAIN CONSTANTS --- */
float kp = 0;
float ki = 0;
float kd = 0;
float kv = 0;
/* -------------------------- */

/* -- FUNCTION PROTOTYPES --- */
void get_accel(void);
void get_gyro(void);

int main(void){
	struct timespec time_tag;
	struct termios oldtio, newtio;
	
	/* --- DATALOGGING INIT --- */
	#ifdef DATALOG_ON
		ofstream fs ("data.txt");
		int datalog_count = 0;
	#endif
	
	/* --- I2C INIT --- */
	open_bus();
	
	/* --- ACCELEROMETER --- */
	set_address(ACCEL_ADDRESS);
	buffer[0] = 0x10;
	buffer[1] = 0x80;
	write_i2c(buffer, 2);
	cout << "Accelerometer Control bytes written" << endl;
	
	/* --- GYRO --- */
	buffer[0] = 0x11;
	buffer[1] = 0x80;
	write_i2c(buffer, 2);
	cout << "Gyro control bytes written" << endl;
	
	/* --- SERIAL INIT --- */
	int msp_fs = open(MSP_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
	if(msp_fs < 0) { cout <<"Error opening MSP serial port" << endl; return -1; }
	else cout << "MSP serial port opened successfully!" << endl;
	
	/* --- SERIAL CONFIG --- */
	tcgetattr(msp_fs, &oldtio);
	bzero(&newtio, sizeof(newtio));
	
	newtio.c_cflag = MSP_BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  	newtio.c_iflag = IGNPAR | ICRNL;
  	newtio.c_oflag = 0;
  	newtio.c_lflag = ICANON;
  	
  	tcflush(msp_fs, TCIFLUSH);
  	tcsetattr(msp_fs, TCSANOW, &newtio);
  	
  	/* --- SETUP MOVING BUFFERS --- */
  	int AIdx = 0;
  	int GIdx = 0;
  	
  	while(1){
  		/* --- I2C READS --- */
  		get_accel();
  		get_gyro();
  		
  		/* --- MOVING BUFFERS ACCEL --- */
  		accel_sma_buf[0][AIdx] = accel[0]; //accel is an array of ints
  		accel_sma_buf[1][AIdx] = accel[1];
  		accel_sma_buf[2][AIdx] = accel[2];
  		AIdx++;
  		if(AIdx >= ACCEL_SMA_PERIOD) AIdx = 0;
  		
  		for(int j=0; j<ACCEL_SMA_PERIOD; j++){
		  accel_tot[0] += accel_sma_buf[0][j];
		  accel_tot[1] += accel_sma_buf[1][j];
		  accel_tot[2] += accel_sma_buf[2][j];
  		}
  		
  		accel_val[0] = (2.0*(accel_tot[0]/ACCEL_SMA_PERIOD))/32767.0;
  		accel_val[1] = (2.0*(accel_tot[1]/ACCEL_SMA_PERIOD))/32767.0;
  		accel_val[2] = (2.0*(accel_tot[2]/ACCEL_SMA_PERIOD))/32767.0;
  		
  		/* --- MOVINF BUFFERS GYRO --- */
  		gyro_sma_buf[GIdx] = gyro; //an int
  		GIdx++;
  		if(GIdx >= GYRO_SMA_PERIOD) GIdx = 0;
  		
  		for(int k=0; k<GYRO_SMA_PERIOD; k++){
		  gyro_tot += gyro_sma_buf[k];
  		}
  		
  		gyro_val = (245.0 *(gyro_tot/GYRO_SMA_PERIOD))/32767.0;
  		  		
  		/* -- ANGLE APPROXIMATION --- */
  		tilt_angle = (-atan2(accel_val[1], sqrt(pow(-accel_val[2],2) + pow(-accel_val[0],2)))*180.0)/M_PI;

  		/* --- CFANGLE APPROXIMATION --- */
  		cf = (ALPHA * (cf_old + (gyro_old*DT))) + ((1.0 - ALPHA) * tilt_angle);
  		cf = cf + ANGLE_OFFSET;
  		 		
  		/* --- PID EQUATIONS --- */
  		pid_p = kp*cf;
  		pid_i = 0; 				//check this equation
  		pid_d = kd*gyro_old;
  		pid_v = 0; 				// check this one too
  		
  		/* --- DEADBAND DO NOTHING --- */
  		if(cf > DEADBAND || cf < -DEADBAND){
		  pwm = -(pid_p + pid_i + pid_d + pid_v);
  		}
  		
  		/* --- PWM OFFSET --- */
  		if(pwm >= 0) pwm = pwm + PWM_MINIMUM;
  		else if(pwm < 0) pwm = pwm - PWM_MINIMUM;
  		
  		if(pwm > 127) pwm = 127;
  		if(pwm < -127) pwm = -127;
  		  		
  		/* --- UART TO MSP --- */
  		unsigned char msp_data[2];
  		
  		if(pwm > 0){
		  msp_data[0] = (127 - pwm);
		  msp_data[0] &= ~0x80;
  		}
  		else{
		  msp_data[0] = -(127 - pwm);
		  msp_data[0] |= 0x80;
  		}
  		
  		msp_data[1] = 0; 		// null char to terminate string
  		
  		int res = write(msp_fs, msp_data, sizeof(msp_data)-1);
  		cout << res << " Bytes written to MSP" << endl;
  		
  		/* --- DATA LOGGING --- */
  		#ifdef DATALOG_ON
		if(fs.is_open()){
		  fs << datalog_count << "\t";
		  fs << tilt_angle << "\t";
		  fs << cf << "\t";
		  fs << gyro_val << "\t";
		  fs << DT << endl;
		}
		datalog_count++;
  		#endif
  		 		
  		/* --- UPDATE OLD VALUES --- */
  		gyro_old = gyro_val;
  		cf_old = cf;
  	}
  	#ifdef DATALOG_ON
	fs.close();
  	#endif
  	
  	tcsetattr(msp_fs, TCSANOW, &oldtio);
  	close(msp_fs);
  	return -1;
} //end main

/* --- GET ACCELEROMETER VALUES --- */
void get_accel(void){
	unsigned char out[6];
	
	set_address(ACCEL_ADDRESS);
	out[0] = 0x28;
	write_i2c(out, 1);
	read_i2c(out, 6);
	
	accel[0] = (short)(out[1] << 8 | out[0]);
	accel[1] = (short)(out[3] << 8 | out[2]);
	accel[2] = (short)(out[4] << 8 | out[4]);
}

/* --- GET GYRO Z VALUE --- */
void get_gyro(void){
	unsigned char out[2];
	
	set_address(ACCEL_ADDRESS);
	out[0] = 0x26;
	write_i2c(out, 1);
	read_i2c(out, 2);
	
	gyro = (short)(out[1] << 8 | out[0]); //only z value of gyro
}
