#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int file_i2c;
int length;
unsigned char buffer[60] = {0};

// --- OPEN DEVICE I2C BUS ---
void open_bus(){
  char *filename = (char*)"/dev/i2c-2"; //i2c-1 for RPi, i2c-2 for BBB
  if((file_i2c = open(filename, O_RDWR)) < 0){
    std::cout << "Failed to open the i2c bus." << std::endl;
  }
}

// --- SETS I2C SLAVE ADDRESS ---
void set_address(int addr){
  if(ioctl(file_i2c, I2C_SLAVE, addr) < 0){
    std::cout << "Failed to acquire bus access and/or talk to the slave." << std::endl;
  }
}

// --- WRITE BYTES TO I2C BUS ---
void write_i2c(unsigned char* write_buf, int length){
  if(write(file_i2c, write_buf, length) != length){
    std::cout << "Failed to write to the i2c bus." << std::endl;
  }
}

// --- READ BYTES FROM I2C BUS ---
void read_i2c(unsigned char* read_buf, int length){
  if(read(file_i2c, read_buf, length) != length){
    std::cout << "Failed to read from the i2c bus." << std::endl;
  }
}
