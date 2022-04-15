#!/usr/bin/env python3
# found in navigation

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

useLidar = True #For debugging
useCamera = True
useMovement = True


foldername = "April28Test1"
root = "../" + foldername + "/"

lidar_filename = root + "lidar.txt"
imu_raw_filename = root + "imu_raw.txt"
imu_normal_filename = root + "imu_normal.txt"
imu_state_filename = root + "imu_state.txt"
camera_filename = root + "camera.txt"

if (not os.path.exists(root)):
    os.makedirs(root)

lidar_file = open(lidar_filename, "w")
imu_raw_file = open(imu_raw_filename, "w")
imu_normal_file = open(imu_normal_filename, "w")
imu_state_file = open(imu_state_filename, "w")
camera_file = open(camera_filename, "w")
writer = csv.writer(lidar_file)

#initialization
i2c = board.I2C()  # uses board.SCL and board.SDA
sox = LSM6DS33(i2c, address=0x6b)
camera_ser = serial.Serial(port="/dev/ttyAMA0", baudrate=9600,
                           bytesize=serial.EIGHTBITS,
                           stopbits=serial.STOPBITS_ONE,
                           timeout=0)
t_start_calibrate = time.time()
state = np.array((0, 0, 0, 0, 0, 0, 0)) #time, velx, vely, velz, gyro angles xyz

#write file headers
writer.writerow("LiDAR','Distance (m)', 'Time (s)\n")
imu_raw_file.write("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n")
imu_normal_file.write("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ\n")
imu_state_file.write("timestamp,velX,velY,velZ,gyroXdrift,gyroYdrift,gyroZdrift\n")
camera_file.write("timestamp, positionChange\n")


#gets the IMU measurement subtract gravity and other forces
def get_imu_measurement():
    values = (time.time() - t_start_calibrate,) + sox.acceleration + sox.gyro #had t_start_calibrate before
    
    imu_raw_file.write("%f,%f,%f,%f,%f,%f,%f\n"%(values))
    imu_state_file.write("%f,%f,%f,%f,%f,%f,%f\n"%((time.time()-t_start_calibrate,) + tuple(state[1:]))) #had t_start_calibrate
    
    #rot = Rotation.from_euler('xyz', [state[4], state[5], state[6]], degrees=False)
    rot = Rotation.from_euler('xyz', [0, 0, 0], degrees=False)
    gravity = rot.apply([0, 0, GRAV])
    acc = tuple(np.array(sox.acceleration) - gravity)
    measurement = np.array(acc + sox.gyro)
    return measurement
    
def get_lidar_measurement():
    line = ser.readline().decode('utf-8').rstrip()
    newLine = line.split(',')
    return newLine

def get_camera_measurement():#if no data empty string
    received = camera_ser.readline()
    camera_data = received.decode("ascii")
    return camera_data


def write_movement(cmd):
    
    if(True):
        ser1.write(cmd)
        line2 = ser1.readline().decode('utf-8').rstrip()
        print(line2)
        #time.sleep(1)
        
    return

#calibration for ~10 second. KEEP STILL
calibrationarr = np.empty([0, 6])

values = get_imu_measurement()
while time.time() - t_start_calibrate < 1.0: # TEMPOARYILY MAKING THESE 1 SECOND FOR DEBUGGINS SAKE
    calibrationarr = np.vstack((calibrationarr, values))
    values = get_imu_measurement()
offset = np.median(calibrationarr, axis=0)
print("Done calibrating at %f. Data capture begins."%(time.time()-t_start_calibrate))


#only starts writing data after 1 second
try:
    prev_time = values[0]
    if (useLidar):
        # TODO: This address occasionally changes when we plug/unplug the arduino
        # If this isn't working run "ls /dev/ttyA*" and use that address
        # It will NOT be AMA0
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
        ser.reset_input_buffer()
        
    if (useMovement):
        ser1 = serial.Serial('/dev/ttyACM1', 9600, timeout=0)
        ser1.reset_input_buffer()
        
    j = 0
    turn_detected = 0
    obstacle = False

    t0 = time.time()
    while True:
        j+=1
        current_timestamp = time.time() - t0
        #print(current_timestamp)
        
                
        print("about to collect imu");
        print(time.time() - t0);
        #collect imu normal data
        values = get_imu_measurement() - offset
        #state = values * (values[0]-prev_time) + state
        imu_normal_file.write(str(current_timestamp) + ",")
        for i in range(5):
            imu_normal_file.write(str(values[i]) + ",")
        imu_normal_file.write(str(values[5]) + "\n")
        #prev_time = values[0]
        
        #collect lidar data
        
        print("about to collect lidar");
        print(time.time() - t0);
        
        if (useLidar and ser.in_waiting > 0):
            newLine = get_lidar_measurement()
            newLine.append(current_timestamp)
            print(newLine)
            #lidar_file.write(newLine)
            
           # newLine.append(current_timestamp)
            writer.writerow(newLine)
            
            if(newLine[0] == '2' and float(newLine[1]) <= .4):
                turn_detected += 1
                if turn_detected == 4:
                    write_movement(b"back,1,1.5,200\n")
                    write_movement(b"turn,0,060,200\n")
                    #3write_movement(b"forw,1,1.5\n")
                    turn_detected = 0
                    obstacle = True
            
                    ser1.reset_input_buffer()
                    ser1.reset_output_buffer()
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    
                    #Potentially nest this so that if there's still an object the bot turns back the other way 
                    
            elif(newLine[0] == '2' and float(newLine[1]) >= .41):
                 if(obstacle):
                     write_movement(b"forw,2,1.5,200\n")
                 else:
                     write_movement(b"forw,1,1.5,200\n")
                 if(obstacle):
                     write_movement(b"turn,1,070,200\n")
                     obstacle = False
                     
                 if (ser.in_waiting > 0):
                     newLine = get_lidar_measurement()
                     newLine.append(time.time() - t0)
                         #newLine.append(current_timestamp)
                     print(newLine)
                     writer.writerow(newLine)
                     
                 ser1.reset_input_buffer()
                 ser1.reset_output_buffer()
                 ser.reset_input_buffer()
                 ser.reset_output_buffer()
                     
                     
                     
        #get camera data
                             
        
        if (useCamera):
            print("about to collect camera");
            print(time.time() - t0);
            camera_data = get_camera_measurement()
            
            if (camera_data != ""):
                print("cam;klsdfaj;f")
                #print(camera_data)
                camera_file.write(str(current_timestamp) + "," + str(camera_data))



            
        
except KeyboardInterrupt: #Must use keyboard interrupt (Ctrl + C) to terminate properly
    camera_ser.close()
    lidar_file.close()
    imu_raw_file.close()
    imu_normal_file.close()
    imu_state_file.close()
    camera_file.close()
    print("Data collection successfully terminated. All files closed.")
    sys.exit(0)

