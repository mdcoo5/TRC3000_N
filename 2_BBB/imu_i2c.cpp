#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int file_i2c;
int length;
unsigned char buffer[60] = {0};

void open_bus(){ //---------OPEN I2C BUS------------
  char *filename = (char*)"/dev/i2c-2"; //i2c-1 for RPi, i2c-2 for BBB
		if((file_i2c = open(filename, O_RDWR)) < 0){
			std::cout << "Failed to open the i2c bus." << std::endl;
		}
}

void set_address(int addr){ //----------SETS I2C SLAVE ADDRESS --------------
	if(ioctl(file_i2c, I2C_SLAVE, addr) < 0){
		std::cout << "Failed to acquire bus access and/or talk to the slave." << std::endl;
	}
}

void write_i2c(unsigned char* write_buf, int length){
	if(write(file_i2c, write_buf, length) != length){
		std::cout << "Failed to write to the i2c bus." << std::endl;
	}
}

void read_i2c(unsigned char* read_buf, int length){
	if(read(file_i2c, read_buf, length) != length){
		std::cout << "Failed to read from the i2c bus." << std::endl;
	}
}

int main(void) {
  open_bus();
  set_address(0x6B);

  // --- WRITE CONTROL BYTES ---
  buffer[0] = 0x10;
  buffer[1] = 0x80;
  write_i2c(buffer, 2);
  printf("Control bytes written\n");


  while(1) {
    // --- READ VALUES ---
    buffer[0] = 0x28; //Initial read address
    write_i2c(buffer, 1);
    read_i2c(buffer, 6);
    
    // --- INTERPRET VALUES ---
    for(int j=0; j<6; j++) {
      //printf("Data recieved: %x\n",buffer[j]);
    }

    int Ax = (short)(buffer[1] << 8 | buffer[0]);
    int Ay = (short)(buffer[3] << 8 | buffer[2]);
    int Az = (short)(buffer[5] << 8 | buffer[4]);

    printf("%6d %6d %8d\n", Ax, Ay, Az);
    usleep(1000000);
  }
  return 1;
} //end main
