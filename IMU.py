import time
import board
import signal
import sys
import numpy as np
import math

# first lengthen bias period
# second change measurement to take into account gravity
# thing about coriallis effect?


from adafruit_lsm6ds.lsm6ds33 import LSM6DS33

fileoutput = "../data/z_1_" + input("Enter Filename: ") + ".txt"
GRAV = 9.81



i2c = board.I2C()  # uses board.SCL and board.SDA
sox = LSM6DS33(i2c, address=0x6b)
tstart = time.time()
state = np.array((0, 0, 0, 0, 0, 0, 0)) 

#gets the IMU measurement subtract gravity and other forces
def get_measurement():
    measurement = np.array((time.time()-tstart,) + sox.acceleration + sox.gyro)
    gravity = np.array((0, GRAV*math.sin(state[4]), GRAV*math.cos(state[4])*math.sin(state[5]), GRAV*math.sin(state[4])*math.sin(state[5]), 0, 0, 0))
    return measurement - gravity
    


#calibration for ~10 second. KEEP STILL
calibrationarr = np.empty([0, 6])
values = get_measurement()

while values[0] < 10:
    calibrationarr = np.append(calibrationarr, values)
    values = get_measurement()

offset = np.median(calibrationarr, axis=0)
print("Done calibrating at %f. Data capture begins."%(time.time()-tstart))




file = open(fileoutput, "w")
#print header to file
file.write("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n")
#only starts writing data after 1 second
prev_time = values[0]
try:
    while True:
        #write the accelatation data to the columns 2-4
        #write the gyro data to 5-7
        #write the time from start in 1
        
        values = get_measurement() - offset
        state = values*(values[0]-prev_time) + state
        for i in range(6):
            file.write(str(values[0]) + ",")
        file.write(str(values[6]) + "\n")
        prev_time = values[0]
except KeyboardInterrupt:
    print("program stoped")
    file.close()
    sys.exit(0)