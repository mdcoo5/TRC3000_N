#include "i2c_function_decs.h"
#include <iostream>
#include 

int main(){
  open_bus();
  set_address(0x70);

  //setup of thingy
  buffer[0] = 0x00;
  buffer[1] = 0x21;
  buffer[2] = 0x04;
  write_i2c(buffer, 3);

  float duty_ratio = 0.5;
  while(1){
    int on = 1;
    int off = (int)4096*duty_ratio - 1;

    buffer[0] = 0x06; //address for first pwm
    buffer[1] = 0x01; //LSB for low 1
    buffer[2] = 0x00; //MSB = others 1
    buffer[3] = off & 0xFF; // high 1
    buffer[4] = (off & 0x0F) >> 8; //high 2
    write_i2c(buffer, 5);
  }
}
