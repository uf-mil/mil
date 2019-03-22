#!/usr/bin/env python
import serial
from utils import Packet, CommandPacket
from simulation import SimulatedUSBtoCAN
from threading import Lock


class USBtoCANBoard(object):
    '''
    Driver for USB to CAN board
    '''
    def __init__(self, port, baud=9600, simulated=False, **kwargs):
        self.lock = Lock()
        if simulated:
            self.ser = SimulatedUSBtoCAN(**kwargs)
        else:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=1., **kwargs)

    def request_data(self, device_id, length):
        req = CommandPacket.create_receive_packet(device_id, length)
        with self.lock:
            self.ser.write(req.to_bytes())
            res = Packet.read_packet(self.ser, length)
            return res.payload

    def send_data(self, device_id, data):
        p = CommandPacket.create_send_packet(device_id, data)
        with self.lock:
            self.ser.write(p.to_bytes())
