import smbus
import os
import time

accel_addr = 0x6B #Address for LSM6DS33 Gyro and Accelerometer module
magne_addr = 0x1E #Address for LIS3MDL Magnetometer module

i2c = smbus.SMBus(1) # 1 for RPi i2c bus 1

def i2c_write(addr, byte):
    i2c.write_byte(addr, byte)
    return

def i2c_write_target(addr, target, byte):
    i2c.write_byte_data(addr, target, byte)
    return

def i2c_write_block(addr, target, data): #data is list
    i2c.write_i2c_block_data(addr, target, data)
    return

def i2c_read_block(addr, target, length):
    out = []
    out = i2c.read_i2c_block_data(addr, target)
