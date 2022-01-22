import time
import board
import signal
import sys
import numpy as np

from adafruit_lsm6ds.lsm6ds33 import LSM6DS33

fileoutput = "../data/z_1_" + input("Enter Filename: ") + ".txt"


i2c = board.I2C()  # uses board.SCL and board.SDA
sox = LSM6DS33(i2c, address=0x6b)

tstart = time.time()
#calibration for ~1 second. KEEP STILL
calibrationarr = np.empty([0, 6])
for i in range(100):
    values = tuple(sox.acceleration + sox.gyro)
    calibrationarr = np.vstack([calibrationarr, values])

offset = np.median(calibrationarr, axis=0)
print("Done calibrating at %f. Data capture begins."%(time.time()-tstart))




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
        values = (time.time() - t0,) + tuple(map(lambda i, j: i - j, sox.acceleration + sox.gyro, offset))
        file.write("%f,%f,%f,%f,%f,%f,%f\n"%(values))
except KeyboardInterrupt:
    print("program stoped")
    file.close()
    sys.exit(0)