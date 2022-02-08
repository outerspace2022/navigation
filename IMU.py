import time
import board
import signal
import sys
import numpy as np
import math
from scipy.spatial.transform import Rotation

# first lengthen bias period
# second change measurement to take into account gravity
# thing about coriallis effect?


from adafruit_lsm6ds.lsm6ds33 import LSM6DS33

fileoutput = "../data4/integration" + input("Enter Filename: ") + ".txt"
#the IMU is upside down so gravity is positive 
GRAV = 9.81



i2c = board.I2C()  # uses board.SCL and board.SDA
sox = LSM6DS33(i2c, address=0x6b)
tstart = time.time()
state = np.array((0, 0, 0, 0, 0, 0, 0))
PI = math.pi

#gets the IMU measurement subtract gravity and other forces
def get_measurement():
    
    rot = Rotation.from_euler('xyz', [state[4], state[5], state[6]], degrees=False)
    gravity = rot.apply([0, 0, GRAV])
    print("X: " + str((state[4]*180)/PI))
    print("Y: " + str((state[5]*180)/PI))
    print("Z: " + str((state[6]*180)/PI))
    print(gravity)
    print(sox.acceleration)

    acc = tuple(np.array(sox.acceleration) - gravity)
    print(acc)
    measurement = np.array((time.time()-tstart,) + acc + sox.gyro)
    return measurement
    


#calibration for ~10 second. KEEP STILL
calibrationarr = np.empty([0, 7])
values = get_measurement()
while values[0] < 10:
    calibrationarr = np.vstack((calibrationarr, values))
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
        
        values = get_measurement()
        state = values*(values[0]-prev_time) + state
        for i in range(6):
            file.write(str(values[i]) + ",")
        file.write(str(values[6]) + "\n")
        prev_time = values[0]
except KeyboardInterrupt:
    print("program stoped")
    file.close()
    sys.exit(0)