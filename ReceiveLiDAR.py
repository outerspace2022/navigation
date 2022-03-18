#!/usr/bin/env python3



# Robotics.com Raspberry Pi Serial Communication
import serial
import csv
import time
from datetime import datetime


fields = ['LiDAR','Distance (m)', 'Time (s)']

if __name__ == '__main__':
    
    #print('here1')
    with open(r'SampleOutput1.csv', 'a+') as f:
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
                newLine.append(time.time())
                #newLine.append(datetime.now().strftime('%M:%S.%f'))
                writer.writerow(newLine)
                print(newLine)

