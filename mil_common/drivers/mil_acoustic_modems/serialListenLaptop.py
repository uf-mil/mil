#!/usr/bin/env python                                                  |
# -*- coding: utf-8 -*-                                                |
#                                                                      |
#  serialComm.py                                                       |
#                                                                      |
#  Copyright 2020 Connor Bolick                                        |
#                                                                      |
#  This program is free software; you can redistribute it and/or modify|
#  it under the terms of the GNU General Public License as published by|
#  the Free Software Foundation; either version 2 of the License, or   |
#  (at your option) any later version.                                 |
#                                                                      |
#  This program is distributed in the hope that it will be useful,     |
#  but WITHOUT ANY WARRANTY; without even the implied warranty of      |
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the       |
#  GNU General Public License for more details.                        |
#                                                                      |
#  You should have received a copy of the GNU General Public License   |
#  along with this program; if not, write to the Free Software         |
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,          |
#  MA 02110-1301, USA.                                                 |
#                                                                      |
#______________________________________________________________________|

import serial
import time

def main(args):
	filename = "reciever_45_90.csv"
	#setup serial object
	ser = serial.Serial('COM7',
			baudrate=19200,
			parity = serial.PARITY_NONE,
			stopbits = serial.STOPBITS_ONE,
			bytesize = serial.EIGHTBITS)
		
	#print(ser.read(64).decode('utf-8')
	#print(ser.write(b'Hello World'))
	time.sleep(3)
	#main code body here
	running = True
	try:	
		while(running):
			with open(filename,'a') as csvfile:
				data = ser.read(64).decode('utf-8')
				#print(data)
				csvfile.write(data)	
	except KeyboardInterrupt:
		print("Exiting program now")
	finally:
		ser.close()
		pass
	return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))

