#!/usr/bin/env python
import serial
from utils import ReceivePacket, CommandPacket
from simulation import SimulatedUSBtoCAN
from threading import Lock


class USBtoCANBoard(object):
    '''
    ROS-independent wrapper which provides an interface to connect to the USB to CAN board
    via a serial (or simulated serial) device. Provides thread-safe funtionality.
    '''
    def __init__(self, port, baud=9600, simulated=False, **kwargs):
        '''
        Creates an instance.
        @param port: path to serial device, such as /dev/ttyUSB0
        @param baud: baud rate of serial device to connect to
        @param simulated: If true, use the simulated serial device rather than connecting to the real one
        '''
        self.lock = Lock()
        if simulated:
            self.ser = SimulatedUSBtoCAN(**kwargs)
        else:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=5., **kwargs)

    def request_data(self, device_id, length):
        '''
        Request bytes from a CAN device
        Note: write/read operation is mutex locked
        @param device_id: CAN device ID to request data from
        @param length: the number of bytes to request
        @return: the data retrieved from the device, which should be of the specified length
        '''
        req = CommandPacket.create_request_packet(device_id, length)
        with self.lock:
            # print 'Requesting {} bytes from device {}: {}'.format(length, device_id, hexify(req.to_bytes()))
            self.ser.write(req.to_bytes())
            res = ReceivePacket.read_packet(self.ser, length)
            return res.data

    def send_data(self, device_id, data):
        '''
        Sends data to a CAN device
        Note: write operation is mutex locked
        @param device_id: CAN device ID to send data to
        @param data: bytes/string object of the data to send to the device
        '''
        p = CommandPacket.create_send_packet(device_id, data)
        with self.lock:
            # print "Sending '{}' to device {}: {}".format(data, device_id, hexify(p.to_bytes()))
            self.ser.write(p.to_bytes())
