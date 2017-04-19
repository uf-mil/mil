#!/usr/bin/env python
import serial
import time
import sys
import mil_misc_tools

__author__ = 'Daniel Dugger'

PORT = '/dev/serial/by-id/usb-MIL_Data_Merge_Board_Ports_5_to_8_DMBP58-if00-port0'

fprint = mil_misc_tools.FprintFactory(title='ActuatorTester').fprint

ser = None
try:
  ser = serial.Serial(PORT, baudrate=9600)
except serial.serialutil.SerialException as e:
  fprint(e)
  sys.exit()

fprint("Using serial resource: " + ser.name)

main_menu = "(1) Open & Close Valve, (2) Open Valve, (3) Close Valve, (4) Ping, (9) Exit: "
valve_menu = "Enter valve number to {} (1-12): "
delay_menu = "Enter delay between valve open and close (seconds): "

def finput(prompt):
    '''Get input with fancy prompt'''
    fprint(prompt)
    return input()

while True:
    print "fuck"
    option = finput(main_menu)
    if option == 1:
        valveNumber = finput(valve_menu.format('open and close'))
        valveDelay = finput(delay_menu)
        if valveNumber >= 1 and valveNumber <=12:
            valveCommand = int('20', 16) + valveNumber
            valveChecksum = valveCommand ^ int('ff', 16)
            ser.write(bytearray([valveCommand, valveChecksum]))
            reply = ser.read(2)
            if reply == '\x01\xFE':
                fprint("Valve opened")
            else:
                fprint("Unknown reply")
            time.sleep(valveDelay)
            valveCommand2 = int('30', 16) + valveNumber
            valveChecksum2 = valveCommand2 ^ int('ff', 16)
            ser.write(bytearray([valveCommand2, valveChecksum2]))
            reply2 = ser.read(2)
            if reply2 == '\x00\xFF':
                fprint("Valve closed")
            else:
                fprint("Unknown reply")
    elif option == 2:
        valveNumber = finput(valve_menu.format('open'))
        if valveNumber >= 1 and valveNumber <=12:
            valveCommand = int('20', 16) + valveNumber
            valveChecksum = valveCommand ^ int('ff', 16)
            ser.write(bytearray([valveCommand, valveChecksum]))
            reply = ser.read(2)
            if reply == '\x01\xFE':
                fprint("Valve opened")
            else:
                fprint("Unknown reply")
        else:
            print "Valve number must be between 1 and 12)"
    elif option == 3:
        valveNumber = finput(valve_menu.format('close'))
        if valveNumber >= 1 and valveNumber <=12:
            valveCommand = int('30', 16) + valveNumber
            valveChecksum = valveCommand ^ int('ff', 16)
            ser.write(bytearray([valveCommand, valveChecksum]))
            reply = ser.read(2)
            if reply == '\x00\xFF':
                fprint("Valve closed")
            else:
                fprint("Unknown reply")
        else:
            fprint("Valve number must be between 1 and 12")
    elif option == 4:
        ser.write("\x10\xEF")
        pingReply = ser.read(2)
        fprint(pingReply.encode('hex'))
        if pingReply == "\x11\xEE":
            fprint("Ping Reply OK")
        else:
            fprint("Ping Reply BAD")
    elif option == 9:
        sys.exit()
    else:
        fprint("Unsupported Option")

