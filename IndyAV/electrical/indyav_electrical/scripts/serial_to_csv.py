#!/usr/bin/env python2
## This script reads lines from arduino serial communication and saves it to 'output.txt'
## Requires package: pyserial

# TODO rewrite this in a proper c++ ros sensor driver

import serial
import csv
from time import sleep
import rospy
import std_msgs


serial = serial.Serial("/dev/ttyACM0",115200,timeout = 1); # Open serial port at required baude rate
#f = open("output.txt","w");                         # open txt file to write
print(serial);                                      # show serial port info on terminal
#time.sleep(5);                                      # wait 5 sec

# Serial readline loop
'''
with open('output.csv', 'w') as new_file:
        csv_writer = csv.writer(new_file,delimiter = ',')
        csv_writer.writerow(["time","steeringPot","breakPed","accPed","brakePressure",
                             "rearSpeed","passSpeed","driverSpeed"])
'''
while True:
    data = serial.readline().strip();
    '''
    try:
        data = data.decode('utf-8'); # decode utf-8 unicode characters as str
    except UnicodeDecodeError as e:
        print(data)
    '''
    print(data);                 # show on terminal
    #f.write(data);               # write to txt file 
    data = data[:(len(data))]
    #csv_writer.writerow([data])           
