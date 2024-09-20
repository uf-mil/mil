#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from threading import Event
from typing import Union

import rospy
from electrical_protocol import Packet, ROSSerialDevice
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


@dataclass
class RequestAddPacket(Packet, class_id=0x37, subclass_id=0x00, payload_format="ff"):
    number_one: float
    number_two: float


@dataclass
class RequestSubPacket(Packet, class_id=0x37, subclass_id=0x01, payload_format="ff"):
    start: float
    minus: float


@dataclass
class AnswerPacket(Packet, class_id=0x37, subclass_id=0x02, payload_format="f"):
    result: float


class CalculatorDevice(
    ROSSerialDevice[Union[RequestAddPacket, RequestSubPacket], AnswerPacket],
):
    def __init__(self):
        self.port_topic = rospy.Subscriber("~port", String, self.port_callback)
        self.start_service = rospy.Service("~trigger", Empty, self.trigger)
        self.answer_topic = rospy.Publisher("~answer", Float32, queue_size=10)
        self.next_packet = Event()
        self.i = 0
        super().__init__(None, 115200)

    def port_callback(self, msg: String):
        self.connect(msg.data, 115200)

    def trigger(self, _: EmptyRequest):
        self.num_one, self.num_two = self.i, 1000 - self.i
        self.i += 1
        self.send_packet(
            RequestAddPacket(number_one=self.num_one, number_two=self.num_two),
        )
        return EmptyResponse()

    def on_packet_received(self, packet) -> None:
        self.answer_topic.publish(Float32(data=packet.result))
        self.next_packet.set()


if __name__ == "__main__":
    rospy.init_node("calculator_device")
    device = CalculatorDevice()
    rospy.on_shutdown(device.close)
    rospy.spin()
