#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int file_i2c;
int length;
unsigned char buffer[60] = {0};

int open_bus(){ //---------OPEN I2C BUS------------
		char *filename = (char*)"/dev/i2c-1";
		if((file_i2c = open(filename, O_RDWR)) < 0){
			std::cout << "Failed to open the i2c bus." << std::endl;
			return 0;
		}
}

int set_address(int addr){ //----------SETS I2C SLAVE ADDRESS --------------
	if(ioctl(file_i2c, I2C_SLAVE, addr) < 0){
		std::cout << "Failed to acquire bus access and/or talk to the slave." << std::endl;
		return 0;
	}
}

int write_i2c(unsigned char write_buf, int length){
	if(write(file_i2c, write_buf, length) != length){
		std::cout << "Failed to write to the i2c bus." << std::endl;
		return 0;
	}
}

int read_i2c(unsigned char read_buf, int length){
	if(read(file_i2c, read_buf, length) != length){
		std::cout << "Failed to read from the i2c bus." << std::endl;
		return 0;
	}
}

int main(){

	open_bus(); // open i2c bus communication
	set_address(0x77); // <<<<< Sets I2C slave address

	//-------WRITE BYTES------------
	buffer[0] = 0xF4;
	buffer[1] = 0xB5;
	write_i2c(buffer, 2);

	//------WRITE BYTES AGAIN-------
	buffer[0] = 0xF7;
	write_i2c(buffer, 1);

	//-------READ BYTES-------------
  read_i2c(buffer, 6);

	// ------OUTPUT RESULTS --------
	for(int i=0; i<length ;i++){
		printf("Data from register %x : %x\n", (0xF7 + i), buffer[i]);
	}


	int temp_raw = (buffer[3] << 12) | (buffer[4] << 4) | (buffer[5] >> 4);
	int press_raw = (buffer[0] << 12) | (buffer [1] << 4) | (buffer[2] >> 4);
	printf("raw temperature value: %d raw pressure value %d\n", temp_raw, press_raw);

std::cout << "Finished!" << std::endl;
return 0;
}
