#!/usr/bin/env python3

import time
import board
import signal
import sys
import numpy as np
import math
from scipy.spatial.transform import Rotation

from datetime import datetime

# first lengthen bias period
# second change measurement to take into account gravity
# thing about coriallis effect?


from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
name = "trial1"
fileoutput = "../trial" + name + ".txt"
raw_data = "../data4/" + name + "_raw.txt"
raw = open(raw_data, "w")
#print header to file
raw.write("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n")
state_data = "../data4/" + name + "_state.txt"
state_data = open(state_data, "w")

#the IMU is upside down so gravity is positive 
GRAV = 9.81



i2c = board.I2C()  # uses board.SCL and board.SDA
sox = LSM6DS33(i2c, address=0x6b)
tstart = time.time()
#time, velx, vely, velz, 
state = np.array((0, 0, 0, 0, 0, 0, 0))
PI = math.pi


import serial
import csv
import time
from datetime import datetime
fields = ['LiDAR','Distance (m)', 'Time (s)']

def LiDAR(tstart):
    with open(r'SampleOutput1.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        
       #tstart = time.time()
        
        ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
        ser.reset_input_buffer()
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                #print(line)

                newLine = line.split(',')
                newLine.append(time.time() - tstart)
                #newLine.append(datetime.now().strftime('%M:%S.%f'))
                writer.writerow(newLine)
                print(newLine)



#gets the IMU measurement subtract gravity and other forces
def get_measurement():
    values = (time.time() - tstart,) + sox.acceleration + sox.gyro #had tStart before
    
    raw.write("%f,%f,%f,%f,%f,%f,%f\n"%(values))
    state_data.write("%f,%f,%f,%f,%f,%f,%f\n"%((time.time()-tstart,) + tuple(state[1:]))) #had tStart
    
    
    #rot = Rotation.from_euler('xyz', [state[4], state[5], state[6]], degrees=False)
    rot = Rotation.from_euler('xyz', [0, 0, 0], degrees=False)
    gravity = rot.apply([0, 0, GRAV])
    acc = tuple(np.array(sox.acceleration) - gravity)
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

LiDAR(tstart)


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
            file.write(str(values[i]) + ",")
        file.write(str(values[6]) + "\n")
        prev_time = values[0]
except KeyboardInterrupt:
    print("program stoped")
    file.close()
    sys.exit(0)
