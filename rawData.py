import time
import board
import signal
import sys
import numpy as np

from adafruit_lsm6ds.lsm6ds33 import LSM6DS33

fileoutput = "../data4/" + input("Enter Filename: ") + ".txt"


i2c = board.I2C()  # uses board.SCL and board.SDA
sox = LSM6DS33(i2c, address=0x6b)

t0 = time.time()
file = open(fileoutput, "w")
#print header to file
file.write("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n")

#only starts writing data after 1 second
try:
    while True:
        #write the accelatation data to the columns 2-4
        #write the gyro data to 5-7
        #write the time from start in 1
        values = (time.time() - t0,) + sox.acceleration + sox.gyro
        file.write("%f,%f,%f,%f,%f,%f,%f\n"%(values))
except KeyboardInterrupt:
    print("program stoped")
    file.close()
    sys.exit(0)
