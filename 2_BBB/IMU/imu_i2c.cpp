#include <stdio.h>
#include <iostream>
#include <math.h>
#include "i2c_function_decs.h"

#define ACCEL_ADDRESS 0x6B
#define MAG_ADDRESS 0x1E

float accel[3];
float mag[3];
float angle[2];
float heading;

void get_accel(void);
void get_mag(void);

int main(void) {
  open_bus();

  // --- WRITE ACCELEROMETER CONTROL BYTES ---
  set_address(ACCEL_ADDRESS);
  buffer[0] = 0x10;
  buffer[1] = 0x80;
  write_i2c(buffer, 2);
  printf("Accel Control bytes written\n");

  // --- WRITE MAG CONTROL BITS ---
  set_address(MAG_ADDRESS);
  buffer[0] = 0x20;
  buffer[1] = 0x50;  
  write_i2c(buffer, 2);
  buffer[0] = 0x22;
  buffer[1] = 0x00;
  write_i2c(buffer, 2);
  printf("Mag Control bytes written\n");


  while(1) {
    // --- READ ACCELEROMETER VALUES
    get_accel();
    get_mag();

    // --- CONVERSION FOR ANGLE APPROXIMATION ---
    angle[0] = -atan2(accel[1], sqrt(pow(-accel[2],2) + pow(-accel[0],2)));
    angle[1] = -atan2(-accel[2], sqrt(pow(accel[1],2) + pow(-accel[0],2)));

    // --- OUTPUT VALUES ---
    printf("Tilt: %-3.4f\tRoll: %-3.4f\n", (angle[0]*180.0)/M_PI, (angle[1]*180.0)/M_PI);
    printf("Mag x: %-6f\ty:%6f\tz:%6f\n", mag[0], mag[1], mag[2]);

    usleep(5000);
  }
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

void get_mag(void) {
  int Mx, My, Mz;
  unsigned char out[6];

  set_address(0x1E);
  out[0] = 0x27;
  write_i2c(out, 1);
  read_i2c(out, 1);
 if((out[0] & 0x04) != 0x04) return;
  
  out[0] = 0x28;
  write_i2c(out, 1);
  read_i2c(out, 6);

  Mx = (short)(out[1] << 8 | out[0]);
  My = (short)(out[3] << 8 | out[2]);
  Mz = (short)(out[5] << 8 | out[4]);

  mag[0] = (float)((2.0*Mx)/32767.0); //Ax
  mag[1] = (float)((2.0*My)/32767.0); //Ay
  mag[2] = (float)((2.0*Mz)/32767.0); //Az
}
