#!/usr/bin/env python3

import time
import board
import signal
import os
import sys
import numpy as np
import math
from scipy.spatial.transform import Rotation
from datetime import datetime
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
import serial
import csv
import time

GRAV = 9.81 #the IMU is upside down so gravity is positive 
PI = math.pi

useLidar = False #For debugging

foldername = "springbreak"
root = "../" + foldername + "/"

lidar_filename = root + "lidar.txt"
imu_raw_filename = root + "imu_raw.txt"
imu_normal_filename = root + "imu_normal.txt"
imu_state_filename = root + "imu_state.txt"

if (not os.path.exists(root)):
    os.makedirs(root)

lidar_file = open(lidar_filename, "w")
imu_raw_file = open(imu_raw_filename, "w")
imu_normal_file = open(imu_normal_filename, "w")
imu_state_file = open(imu_state_filename, "w")

#initialization
i2c = board.I2C()  # uses board.SCL and board.SDA
sox = LSM6DS33(i2c, address=0x6b)
tstart = time.time()
state = np.array((0, 0, 0, 0, 0, 0, 0)) #time, velx, vely, velz, gyro angles xyz

#write file headers
lidar_file.write("LiDAR','Distance (m)', 'Time (s)")
imu_raw_file.write("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n")
imu_normal_file.write("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n")
imu_state_file.write("timestamp,velX,velY,velZ,gyroXdrift,gyroYdrift,gyroZdrift\n")


#gets the IMU measurement subtract gravity and other forces
def get_imu_measurement():
    values = (time.time() - tstart,) + sox.acceleration + sox.gyro #had tStart before
    
    imu_raw_file.write("%f,%f,%f,%f,%f,%f,%f\n"%(values))
    imu_state_file.write("%f,%f,%f,%f,%f,%f,%f\n"%((time.time()-tstart,) + tuple(state[1:]))) #had tStart
    
    #rot = Rotation.from_euler('xyz', [state[4], state[5], state[6]], degrees=False)
    rot = Rotation.from_euler('xyz', [0, 0, 0], degrees=False)
    gravity = rot.apply([0, 0, GRAV])
    acc = tuple(np.array(sox.acceleration) - gravity)
    measurement = np.array((time.time()-tstart,) + acc + sox.gyro)
    return measurement
    
def get_lidar_measurement():
    line = ser.readline().decode('utf-8').rstrip()
    newLine = line.split(',')
    newLine.append(time.time() - tstart)
    return newLine

#calibration for ~10 second. KEEP STILL
calibrationarr = np.empty([0, 7])
values = get_imu_measurement()
while values[0] < 1.0: # TEMPOARYILY MAKING THESE 1 SECOND FOR DEBUGGINS SAKE
    calibrationarr = np.vstack((calibrationarr, values))
    values = get_imu_measurement()
offset = np.median(calibrationarr, axis=0)
print("Done calibrating at %f. Data capture begins."%(time.time()-tstart))


#only starts writing data after 1 second
try:
    prev_time = values[0]
    if (useLidar):
        ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
        ser.reset_input_buffer()
    while True:
        
        #collect imu normal data
        values = get_imu_measurement() - offset
        state = values * (values[0]-prev_time) + state
        for i in range(6):
            imu_normal_file.write(str(values[i]) + ",")
        imu_normal_file.write(str(values[6]) + "\n")
        prev_time = values[0]
        
        #collect lidar data
        if (useLidar and ser.in_waiting > 0):
            newLine = get_lidar_measurement
            print(newLine)
            lidar_file.write(newLine)


        
except KeyboardInterrupt: #Must use keyboard interrupt (Ctrl + C) to terminate properly
    lidar_file.close()
    imu_raw_file.close()
    imu_normal_file.close()
    imu_state_file.close()
    print("Data collection successfully terminated. All files closed.")
    sys.exit(0)
