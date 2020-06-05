import wiringpi as wpi
import time

i2c_dev = "/dev/i2c-1"
i2c_addr = 0x44
result_reg = 0x00
configuration_reg = 0x01
#configuration = 0b1100101000000000
configuration = 0xC810

wpi.wiringPiSetup()
print("Past setup")
i2c_fd = wpi.wiringPiI2CSetupInterface(i2c_dev, i2c_addr)
print(wpi.wiringPiI2CReadReg16(i2c_fd, 0x7E))
print(wpi.wiringPiI2CReadReg16(i2c_fd, 0x7F))

wpi.wiringPiI2CWriteReg16(i2c_fd, configuration_reg, configuration)
print("Past writing")
for i in range(0, 50):
    conversion_ready = wpi.wiringPiI2CReadReg16(i2c_fd, 0x01)
    if((conversion_ready & (1<<7))>>7):
        time.sleep(1)
        print(wpi.wiringPiI2CReadReg16(i2c_fd, result_reg))
        wpi.wiringPiI2CWriteReg16(i2c_fd, configuration_reg, configuration&~(2<<9))

print("Past reading")
time.sleep(10)
