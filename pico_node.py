#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import rospy
import serial
from mil_usb_to_can.sub9 import Packet
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


class KillStatus(IntEnum):
    MOBO_REQUESTED = 0
    RF_KILL = 1
    EMERGENCY_STOP = 2


@dataclass
class KillSetPacket(Packet, msg_id=0x10, subclass_id=0x00, payload_format="<BB"):
    set: bool
    status: KillStatus


@dataclass
class KillReceivePacket(Packet, msg_id=0x10, subclass_id=0x01, payload_format="<BB"):
    set: bool
    status: KillStatus


class AdrianPicoDriver:
    def __init__(self):
        self.device = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
        self.kill_srv = rospy.Service("~kill", SetBool, self.kill_srv_handler)

    def kill_srv_handler(self, req: SetBoolRequest) -> SetBoolResponse:
        packet = KillSetPacket(req.data, KillStatus.MOBO_REQUESTED)
        resp = self.send_kill(packet)
        return SetBoolResponse(success=resp is not None)

    def send_kill(self, packet: KillSetPacket) -> KillReceivePacket | None:
        """
        Sends a kill to the device via serial.
        """
        tries = 0
        while tries < 5:
            self.device.write(bytes(packet))
            response = self.device.read(10)
            print(response)
            if not response or response[0] != 0x37 or response[1] != 0x01:
                print("Invalid start bits; ignoring")
                tries += 1
            else:
                response_packet = KillReceivePacket.from_bytes(response)
                print(response_packet)
                assert isinstance(response_packet, KillReceivePacket)
                return response_packet
        return None

    def get_packets(self) -> KillReceivePacket:
        byte = None
        while byte != 0x37:
            byte = self.device.read(1)
        packets = byte + self.device.read(9)
        return KillReceivePacket.from_bytes(packets)

    def close(self) -> None:
        print("Shutting down pico driver")
        self.device.read(1000)
        self.device.close()


if __name__ == "__main__":
    rospy.init_node("adrian_pico_driver")
    driver = AdrianPicoDriver()
    rospy.on_shutdown(driver.close)
    print("hi")
    while not rospy.is_shutdown():
        if driver.device.in_waiting >= 10:
            print(driver.get_packets())
