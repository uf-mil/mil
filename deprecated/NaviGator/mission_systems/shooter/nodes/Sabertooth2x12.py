#!/usr/bin/env python
import serial
import math
import struct


class Sabertooth2x12:
    def __init__(self, filename, sim=False):
        self.motor_1_speed = 0.0
        self.motor_2_speed = 0.0
        self.filename = filename
        self.sim = sim
        if not self.sim:
            self.ser = serial.Serial(filename, baudrate=19200)
        else:
            global rospy
            import rospy

    def _map(self, x, in_min, in_max, out_min, out_max):
        return int(math.ceil((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))

    def _contrain(self, x, low, high):
        return max(min(x, high), low)

    def setMotor1(self, speed):
        contrained_num = self._contrain(speed, -1.0, 1.0)
        if (contrained_num == self.motor_1_speed):
            return
        self.motor_1_speed = contrained_num
        if not self.sim:
            send_char = self._map(contrained_num, -1.0, 1.0, 1, 127)
            byts = struct.pack("B", send_char)
            self.ser.write(byts)
            self.ser.flush()
        else:
            rospy.loginfo("SETTING MOTOR 1: {}".format(self.motor_1_speed))

    def getMotor1(self):
        return self.motor_1_speed

    def setMotor2(self, speed):
        contrained_num = self._contrain(speed, -1.0, 1.0)
        if (contrained_num == self.motor_2_speed):
            return
        self.motor_2_speed = contrained_num
        if not self.sim:
            send_char = self._map(contrained_num, -1.0, 1.0, 128, 255)
            byts = struct.pack("B", send_char)
            self.ser.write(byts)
            self.ser.flush()
        else:
            rospy.loginfo("SETTING MOTOR 2: {}".format(self.motor_2_speed))

    def getMotor2(self):
        return self.motor_2_speed


if __name__ == '__main__':
    st = Sabertooth2x12("/dev/ttyUSB0")
    st.setMotor1(-1)
