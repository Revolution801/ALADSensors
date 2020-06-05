from smbus2 import SMBus
import time

i2c_dev = "/dev/i2c-1"
i2c_addr = 0x44
result_reg = 0x00
config_reg = 0x01
config = 0b1100111000000000
#configuration = 0xC800

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
        print(0.01 * (1 << ((reading & 0xF000) >> 12)) * (reading & 0x0FFF), end=' ')
        print("lux")
        
    time.sleep(0.5)
