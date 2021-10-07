import serial 
import time 
from datetime import datetime



def main(args):
	filename = 'sender_45_90.csv'
	ser = serial.Serial('COM7',
						baudrate = 19200,
						parity = serial.PARITY_NONE, 
						stopbits = serial.STOPBITS_ONE,
						bytesize = serial.EIGHTBITS)

	#time.sleep(5*60)
	while(True):
		with open(filename,'a') as f:	
			current_time = datetime.now()
			strTime = str(current_time)
			f.write(strTime)
			f.write("\n")
		ser.write(str.encode(str(current_time)))
		ser.write(str.encode("\n"))
		time.sleep(.1)


if __name__ == '__main__':
	import sys
	sys.exit(main(sys.argv))
