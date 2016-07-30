#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int file_i2c;
int length;
unsigned char buffer[60] = {0};

int main(){

	//--------OPEN i2C BUS---------
	char *filename = (char*)"/dev/i2c-1";
	if((file_i2c = open(filename, O_RDWR)) < 0){
		// ERROR HANDLING: you can check errno to see what went wrong
		std::cout << "Failed to open the i2c bus." << std::endl;
		return 0;
	}

	int addr = 0x77; // <<<<<<< I2C address of the slave device
	if(ioctl(file_i2c, I2C_SLAVE, addr) < 0){
		std::cout << "Failed to acquire bus access and/or talk to the slave." << std::endl;
		// ERROR HANDLING: you can check errno to see what went wrong
		return 0;
	}

	//-------WRITE BYTES------------
	buffer[0] = 0xF4;
	buffer[1] = 0xB5;
	length = 2; //Number of bytes to write
	if(write(file_i2c, buffer, length) != length){
		// ERROR HANDLING: you can check errno to see what went wrong
		std::cout << "Failed to write to the i2c bus." << std::endl;
	}

	//------WRITE BYTES AGAIN-------
	buffer[0] = 0xF7;
	length = 1;
	if(write(file_i2c, buffer, length) != length){
		std::cout << "second write fail" << std::endl;
	}

	//-------READ BYTES-------------
	length = 6; // Number of bytes to read
	if(read(file_i2c, buffer, length) != length){
		// ERROR HANDLING: you can check errno to see what went wrong
		std::cout << "Failed to read from the i2c bus." << std::endl;
	}
		for(int i=0; i<length ;i++){
		printf("Data from register %x : %x\n", (0xF7 + i), buffer[i]);
	}


	int temp_raw = (buffer[3] << 12) | (buffer[4] << 4) | (buffer[5] >> 4);
	int press_raw = (buffer[0] << 12) | (buffer [1] << 4) | (buffer[2] >> 4);
	printf("raw temperature value: %d raw pressure value %d\n", temp_raw, press_raw);

std::cout << "Finished!" << std::endl;
return 0;
}
