import serial
import time
from datetime import datetime

print ("Connecting to USB1")
with serial.Serial('/dev/ttyUSB1', 115200, timeout=1) as ser:
    print(ser.name)
    f = open("SQM_data.txt", "w")
    f.write("# UTC Date & Time, Local Date & Time, Temperature, Counts, Frequency, MSAS\n# YYYY-MM-DDTHH:mm:ss.fff;YYYY-MM-DDTHH:mm:ss.fff;Celsius;number;Hz;mag/arcsec^2\n# END OF HEADER\n")
    while(1):
        ser.write(b'rx')
        s = ser.readline()
        print(s)
        now = datetime.now()
        f.write(str(now))
        f.write(str(s))
        f.write('\n')
        time.sleep(1)

    f.close()
