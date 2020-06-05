from smbus2 import SMBus
import time
import serial
from datetime import datetime

i2c_dev = "/dev/i2c-1"
i2c_addr = 0x44
result_reg = 0x00
config_reg = 0x01
config = 0b1100111000000000
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
data_file = open("sensor_data.dat", "w")
data_file.write("YYYY-MM-DD HH:mm:ss.ffffff,lux,b'r,mag/arcsec^2,Hz,idk,idk,Celcius,GPS\n")

print(hex(config))
i2c = SMBus(1)
#i2c.write_word_data(i2c_addr, config_reg, config)
i2c.write_i2c_block_data(i2c_addr, config_reg, [0b11001110, 0b00000000])
for byte in i2c.read_i2c_block_data(i2c_addr, 0x7E, 2):
    print(hex(byte))
for byte in i2c.read_i2c_block_data(i2c_addr, 0x7F, 2):
    print(hex(byte))
print(hex(i2c.read_word_data(i2c_addr, config_reg)))

while(1):
    conversion_ready = i2c.read_word_data(i2c_addr, config_reg)
    if((conversion_ready & 0x80) == 0x80):
        reading = i2c.read_word_data(i2c_addr, result_reg)
        lux = 0.01 * (1 << ((reading & 0xF000) >> 12)) * (reading & 0x0FFF)
        ser.write(b'rx')
        sqm = ser.readline()
        print("Luminance Reading: ", end='')
        print(lux, end=' ')
        print("lux")
        print("SQM Reading: ", end='')
        print(sqm, end=' ')
        now = datetime.now()
        data_file.write(str(now))
        data_file.write(',')
        data_file.write(str(lux))
        data_file.write(",")
        data_file.write(str(sqm))
        data_file.write(",")
        data_file.write("GPS data here")
        data_file.write('\n')
f.close()
