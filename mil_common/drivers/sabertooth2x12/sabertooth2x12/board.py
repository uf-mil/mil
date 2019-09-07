#!/usr/bin/env python
import serial
import struct
from simulated import SimulatedSabertooth2x12


class Sabertooth2x12:
    def __init__(self, filename, address=128, baudrate=9600, sim=False):
        self.address = address
        self.motor_1_speed = 0.0
        self.motor_2_speed = 0.0
        self.filename = filename
        self.sim = sim
        if not self.sim:
            self.ser = serial.Serial(filename, baudrate=baudrate)
        else:
            self.ser = SimulatedSabertooth2x12()

    @staticmethod
    def make_packet(address, command, data):
        checkum = (address + command + data) & 127
        return struct.pack('BBBB', address, command, data, checkum)

    def send_packet(self, command, data):
        packet = self.make_packet(self.address, command, data)
        self.ser.write(packet)

    def set_motor1(self, speed):
        if speed < 0:
            command = 1
        else:
            command = 0
        data = int(min(1.0, abs(speed)) * 127)
        self.send_packet(command, data)

    def set_motor2(self, speed):
        if speed < 0:
            command = 5
        else:
            command = 4
        data = int(min(1.0, abs(speed)) * 127)
        self.send_packet(command, data)
