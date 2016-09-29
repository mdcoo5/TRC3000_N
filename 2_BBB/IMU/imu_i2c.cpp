#include <stdio.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <time.h>
#include "i2c_function_decs.h"

#define ACCEL_ADDRESS 0x6B
#define MAG_ADDRESS 0x1E

using namespace std;

float accel[3];
float gyro[3];
float gyro_old = 0;
float angle[2];
float CFangle, CFangle_old = 0;
unsigned long time_old = 0;
unsigned long time_new = 0;
float DT;

void get_accel(void);
void get_gyro(void);

int main(void) {
  struct timespec time_tag;
  ofstream fs ("data.txt");
  open_bus();

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
    CFangle = 0.98 * (CFangle_old + gyro_old*(DT)) + 0.02 * accel[0];

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
    gyro_old = gyro[2];
    CFangle_old = CFangle;
    usleep(1000);
  }
  fs.close();
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
