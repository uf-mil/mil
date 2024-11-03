#!/usr/bin/env python3

import os
import pty
import unittest
from dataclasses import dataclass
from enum import Enum, IntEnum

import rospy
import rostest
from electrical_protocol import Packet
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty


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


class SimulatedBasicTest(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)
        self.port_publisher = rospy.Publisher(
            "/calculator_device/port",
            String,
            queue_size=1,
        )
        self.answer_one_subscriber = rospy.Subscriber(
            "/calculator_device/answer_one",
            Float32,
            self.answer_callback_one,
        )
        self.answer_two_subscriber = rospy.Subscriber(
            "/calculator_device/answer_two",
            Float32,
            self.answer_callback_two,
        )
        self.count = 0

    def test_simulated(self):
        self.master, self.slave = pty.openpty()
        serial_name = os.ttyname(self.slave)
        while not self.port_publisher.get_num_connections() and not rospy.is_shutdown():
            print("waiting for port connection...")
            rospy.sleep(0.1)
        self.port_publisher.publish(String(serial_name))
        self.trigger_service_caller = rospy.ServiceProxy(
            "/calculator_device/trigger_one",
            Empty,
        )
        for i in range(1000):
            self.trigger_service_caller()
            packet_bytes = os.read(self.master, 100)
            packet = RequestAddPacket.from_bytes(packet_bytes)
            self.assertEqual(
                packet.number_one,
                i,
                f"packet.number_one: {packet.number_one}, i: {i}",
            )
            self.assertEqual(
                packet.number_two,
                1000 - i,
                f"packet.number_two: {packet.number_two}, i: {i}",
            )
            os.write(
                self.master,
                bytes(AnswerPacket(packet.number_one + packet.number_two)),
            )
        rospy.sleep(2)
        self.assertGreaterEqual(self.count, 900)
        self.trigger_two_service_caller = rospy.ServiceProxy(
            "/calculator_device/trigger_two",
            Empty,
        )
        self.trigger_two_service_caller()
        packet_bytes = os.read(self.master, 100)
        packet = CharacterPacket.from_bytes(packet_bytes)
        self.assertEqual(
            packet.single_char,
            "a",
            f"packet.single_char: {packet.single_char}",
        )
        self.assertEqual(
            packet.big_str,
            "small",
            f"packet.big_str: {packet.big_str}",
        )
        os.write(
            self.master,
            bytes(EnumPacket(Sign.MULTIPLY, CalculatorMode.ADD)),
        )

    def answer_callback_one(self, msg: Float32):
        # at least 900 packets gotten (sometimes lower due to performance)
        self.assertGreaterEqual(msg.data, 900)
        self.count += 1

    def answer_callback_two(self, msg: Float32):
        self.assertEqual(msg.data, 5)

    def tearDown(self):
        os.close(self.master)
        os.close(self.slave)


if __name__ == "__main__":
    rospy.init_node("test_simulated_basic")
    rostest.rosrun("electrical_protocol", "test_simulated_basic", SimulatedBasicTest)
    unittest.main()
