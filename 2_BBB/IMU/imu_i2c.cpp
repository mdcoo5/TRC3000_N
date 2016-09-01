#include <stdio.h>
#include <iostream>
#include <math.h>
#include "i2c_function_decs.h"

float accel[3];
float angle[2];

int main(void) {
  open_bus();
  set_address(0x6B);

  // --- WRITE CONTROL BYTES ---
  buffer[0] = 0x10;
  buffer[1] = 0x80;
  write_i2c(buffer, 2);
  printf("Control bytes written\n");


  while(1) {
    // --- READ ACCELEROMETER VALUES ---
    buffer[0] = 0x28; //Initial read address
    write_i2c(buffer, 1);
    read_i2c(buffer, 6);
    
    // --- INTERPRET VALUES ---
    for(int j=0; j<6; j++) {
      //printf("Data recieved: %x\n",buffer[j]);
    }

    // --- COLLATE RECIEVED VALUES ---
    int Ax = (short)(buffer[1] << 8 | buffer[0]);
    int Ay = (short)(buffer[3] << 8 | buffer[2]);
    int Az = (short)(buffer[5] << 8 | buffer[4]);

    // --- 32767 counts = 2g ---
    accel[0] = (float)((2.0*Ax)/32767.0); //Ax
    accel[1] = (float)((2.0*Ay)/32767.0); //Ay
    accel[2] = (float)((2.0*Az)/32767.0); //Az
    
    // --- CONVERSION FOR ANGLE APPROXIMATION ---
    angle[0] = -atan2(accel[1], sqrt(pow(-accel[2],2) + pow(-accel[0],2)));
    angle[1] = -atan2(-accel[2], sqrt(pow(accel[1],2) + pow(-accel[0],2)));

    /*
    printf("%-7d\t%-7d\t%7d\t", Ax, Ay, Az);
    printf("%-1.5f\t%-1.5f\t%-1.5f\t", accel[0], accel[1], accel[2]);
    */

    // --- OUTPUT VALUES ---
    printf("Tilt: %-3.4f\tRoll: %-3.4f\n", (angle[0]*180.0)/M_PI, (angle[1]*180.0)/M_PI);

    





    usleep(5000);
  }
  return 1;
} //end main

float * get_accel(void) {
  int Ax, Ay, Az;
  unsigned char out[6];

  set_address(0x6B);
  out[0] = 0x28;
  write_i2c(out, 1);
  read_i2c(out, 6);

  Ax = (short)(out[]
