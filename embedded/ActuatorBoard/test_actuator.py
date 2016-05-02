import serial
import time

print "Actuator Test Tool"

ser = serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A603H7W6-if00-port0', baudrate=9600)

print("Using serial resource: " + ser.name)

while True:
    option = input("(1) Open & Close Valve, (2) Open Valve, (3) Close Valve, (4) Ping, (9) Exit: ")
    if option == 1:
        valveNumber = input("Enter valve number to open and close (1-12): ")
        valveDelay = input("Enter delay between valve open and close (seconds): ")
        if valveNumber >= 1 and valveNumber <= 12:
            valveCommand = int('20', 16) + valveNumber
            valveChecksum = valveCommand ^ int('ff', 16)
            ser.write(bytearray([valveCommand, valveChecksum]))
            reply = ser.read(2)
            if reply == '\x01\xFE':
                print "Valve opened"
            else:
                print "Unknown reply"
            time.sleep(valveDelay)
            valveCommand2 = int('30', 16) + valveNumber
            valveChecksum2 = valveCommand2 ^ int('ff', 16)
            ser.write(bytearray([valveCommand2, valveChecksum2]))
            reply2 = ser.read(2)
            if reply2 == '\x00\xFF':
                print "Valve closed"
            else:
                print "Unknown reply"
    elif option == 2:
        valveNumber = input("Enter valve number to open (1-12): ")
        if valveNumber >= 1 and valveNumber <= 12:
            valveCommand = int('20', 16) + valveNumber
            valveChecksum = valveCommand ^ int('ff', 16)
            ser.write(bytearray([valveCommand, valveChecksum]))
            reply = ser.read(2)
            if reply == '\x01\xFE':
                print "Valve opened"
            else:
                print "Unknown reply"
        else:
            print "Valve number must be between 1 and 12)"
    elif option == 3:
        valveNumber = input("Enter valve number to close (1-12): ")
        if valveNumber >= 1 and valveNumber <= 12:
            valveCommand = int('30', 16) + valveNumber
            valveChecksum = valveCommand ^ int('ff', 16)
            ser.write(bytearray([valveCommand, valveChecksum]))
            reply = ser.read(2)
            if reply == '\x00\xFF':
                print "Valve closed"
            else:
                print "Unknown reply"
        else:
            print "Valve number must be between 1 and 12"
    elif option == 4:
        ser.write("\x10\xEF")
        pingReply = ser.read(2)
        print pingReply.encode('hex')
        if pingReply == "\x11\xEE":
            print("Ping Reply OK")
        else:
            print("Ping Reply BAD")
    elif option == 9:
        exit()
    else:
        print "Unsupported Option"
