#!/usr/bin/env python3



# Robotics.com Raspberry Pi Serial Communication
import serial
import csv


fields = ['LiDAR','Distance (m)']

if __name__ == '__main__':
    
    print('here')
    with open(r'SampleOutput.csv', 'a+') as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        print('entering loop')
        
        ser = serial.Serial('/dev/ttyACM2', 9600, timeout=1)
        ser.reset_input_buffer()
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(line)

                newLine = line.split(',')
                writer.writerow(newLine)
                print(newLine)


            