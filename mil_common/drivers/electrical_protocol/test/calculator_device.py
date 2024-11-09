#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from threading import Event
from typing import Union

import rospy
from electrical_protocol import Packet, ROSSerialDevice
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class CalculatorMode(IntEnum):
    ADD = 0
    SUB = 1
    MUL = 2


class Sign(Enum):
    PLUS = "+"
    MINUS = "-"
    MULTIPLY = "*"


@dataclass
class RequestAddPacket(Packet, class_id=0x37, subclass_id=0x00, payload_format="<ff"):
    number_one: float
    number_two: float


@dataclass
class RequestSubPacket(Packet, class_id=0x37, subclass_id=0x01, payload_format="<ff"):
    start: float
    minus: float


@dataclass
class AnswerPacket(Packet, class_id=0x37, subclass_id=0x02, payload_format="<f"):
    result: float


@dataclass
class CharacterPacket(Packet, class_id=0x37, subclass_id=0x03, payload_format="<c10s"):
    single_char: str
    big_str: str


@dataclass
class EnumPacket(Packet, class_id=0x37, subclass_id=0x04, payload_format="<cb"):
    symbol: Sign
    number: CalculatorMode


class CalculatorDevice(
    ROSSerialDevice[
        Union[RequestAddPacket, RequestSubPacket, CharacterPacket],
        Union[AnswerPacket, EnumPacket],
    ],
):
    def __init__(self):
        self.port_topic = rospy.Subscriber("~port", String, self.port_callback)
        self.start_one_service = rospy.Service("~trigger_one", Empty, self.trigger)
        self.start_two_service = rospy.Service("~trigger_two", Empty, self.trigger_two)
        self.answer_one_topic = rospy.Publisher("~answer_one", Float32, queue_size=10)
        self.answer_two_topic = rospy.Publisher("~answer_two", Float32, queue_size=10)
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

    def trigger_two(self, _: EmptyRequest):
        self.send_packet(CharacterPacket("a", "small"))
        return EmptyResponse()

    def on_packet_received(self, packet) -> None:
        if isinstance(packet, AnswerPacket):
            self.answer_one_topic.publish(Float32(data=packet.result))
            self.next_packet.set()
        elif isinstance(packet, EnumPacket):
            self.answer_two_topic.publish(Float32(data=packet.number.value))


if __name__ == "__main__":
    rospy.init_node("calculator_device")
    device = CalculatorDevice()
    rospy.on_shutdown(device.close)
    rospy.spin()
