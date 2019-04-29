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
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1, **kwargs)
        self.ser.flushOutput()
        self.ser.flushInput()

    def read_packet(self):
        '''
        Read a packet from the board, if available. Returns a ReceivePacket instance if one
        was succefully read, or None if the in buffer is empty.
        '''
        with self.lock:
            if self.ser.in_waiting == 0:
                return None
            return ReceivePacket.read_packet(self.ser)

    def send_data(self, data, can_id=0):
        '''
        Sends data to a CAN device
        Note: write operation is mutex locked
        @param device_id: CAN device ID to send data to
        @param data: bytes/string object of the data to send to the device
        '''
        p = CommandPacket.create_send_packet(data, can_id=can_id)
        with self.lock:
            p_bytes = p.to_bytes()
            self.ser.write(p_bytes)
