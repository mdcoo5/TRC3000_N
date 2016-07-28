import time
import os
import smbus

print 'process started!'

i2c = smbus.SMBus(1)

BMP280 = 0x77

def bmp_write_byte(reg, data):
    i2c.write_byte_data(BMP280, reg, data)
    return

def bmp_read_data(reg, length):
    i2c.write_byte(BMP280,reg)
    out = []
    for x in xrange(length)
    out.append(i2c.read_byte(BMP280))

    return out

#initial setup reading config values
bmp_write_byte(0xF4, 0xB5)
config = bmp_read_data(0x88,24)

print config
