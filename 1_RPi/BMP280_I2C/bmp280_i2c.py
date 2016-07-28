import time
import os
import smbus

os.system('clear')
print 'process started!'

i2c = smbus.SMBus(1)
BMP280 = 0x77
t_fine = 0L

def bmp_write(data):
	i2c.write_byte(data)
	return

def bmp_write_byte(reg, data):
    	i2c.write_byte_data(BMP280, reg, data)
    	return

def bmp_read_data(reg, length):
    	i2c.write_byte(BMP280,reg)
    	out = []
    	for x in xrange(length):
    		out.append(i2c.read_byte(BMP280))

    	return out

def bmp_temp_comp(adc_t):
	global t_fine
	var1 = ((adc_t) / 16384.0 - T1 / 1024.0) * T2
	var2 = (((adc_t) / 131072.0 - (T1) / 8192) * ((adc_t) / 131072.0 - (T1) / 8192.0)) * T3
	t_fine = var1 + var2
	T = (var1 + var2)/5120.0
	return T

def bmp_press_comp(adc_p):
	global t_fine
	var1 = long((t_fine/2) - 128000)
	var2 = long(var1 * (var1 * (P6))) 
	var2 = var2 + ((var1 * (P5)) << 17)
	var2 = var2 + ((P4) << 35)
	var1 = ((var1 * var1 * P3) >> 8) + ((var1 * P2) << 12)
	var1 = ((((1) << 47) + var1 )) * (P1) >> 33
	if(var1 == 0.0):
		return 0
	p = long(1048576 - adc_p)
	p = (((p << 31) - var2)*3125) / var1
	var1 = ((P9) * (p>>13) * (p>>13)) >> 25
	var2 = ((P8) * p) >> 19
	p = ((p + var1 + var2) >> 8) + ((P7) << 4)
	return p/256.0

def adc_cov(raw_adc):
	adc_temp = raw_adc[5] >> 4 | raw_adc[4] << 4 | raw_adc[3] << 12
	adc_press = raw_adc[2] >> 4 | raw_adc[1] << 4 | raw_adc[0] << 12
	list = {}
	list['temp'] = float(adc_temp)
	list['press'] = float(adc_press)
	return list

#initial reading config values
config = bmp_read_data(0x88, 24)

T1 = config[1] << 8 | config[2]
T2 = config[3] << 8 | config[2]
T3 = config[5] << 8 | config[4]
P1 = config[7] << 8 | config[6]
P2 = config[9] << 8 | config[8]
P3 = config[11] << 8 | config[10]
P4 = config[13] << 8 | config[12]
P5 = config[15] << 8 | config[14]
P6 = config[17] << 8 | config[16]
P7 = config[19] << 8 | config[18]
P8 = config[21] << 8 | config[20]
P9 = config[23] << 8 | config[22]

print [T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9]


while(1):
	#write setup values to BMP sensor and read adc values from registers
	bmp_write_byte(0xF4, 0xB5)
	raw_adc = bmp_read_data(0xF7, 6)

	#Convert adc vals to raw temp and pressure values
	adc_vals = adc_cov(raw_adc)

	#Convert temp to proper temp
	temp = bmp_temp_comp(adc_vals['temp'])
	print 'temperature:  ',temp,' deg C'
	press = bmp_press_comp(adc_vals['press'])
	print 'air pressure: ',press / 100,' hPa'
	time.sleep(10)
	os.system('clear')
