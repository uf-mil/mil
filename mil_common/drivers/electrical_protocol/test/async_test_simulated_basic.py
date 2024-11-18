#!/usr/bin/env python3
"""
import os
import pty
import unittest
from dataclasses import dataclass

import rospy
import rostest
from electrical_protocol import Packet
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty


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


class AsyncSimulatedBasicTest(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)
        self.port_publisher = rospy.Publisher(
            "/async_calculator_device/port",
            String,
            queue_size=1,
        )
        # print(f"Port topic: {self.port_publisher.resolved_name}")
        self.answer_subscriber = rospy.Subscriber(
            "/async_calculator_device/answer",
            Float32,
            self.answer_callback,
        )
        self.count = 0

    def test_simulated(self):
        # self.port_publisher.publish(String(data="/dev/ttyUSB0"))

        self.master, self.slave = pty.openpty()
        serial_name = os.ttyname(self.slave)
        print(f"serial name: {serial_name}")
        while not self.port_publisher.get_num_connections() and not rospy.is_shutdown():
            print("waiting for port connection...")
            rospy.sleep(0.1)
        self.port_publisher.publish(String(serial_name))

        self.trigger_service_caller = rospy.ServiceProxy(
            "/async_calculator_device/trigger",
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
        self.assertEqual(self.count, 1000)

    def answer_callback(self, msg: Float32):
        self.assertEqual(msg.data, 1000)
        self.count += 1

    def tearDown(self):
        os.close(self.master)
        os.close(self.slave)


if __name__ == "__main__":
    rospy.init_node("async_test_simulated_basic")
    rostest.rosrun(
        "electrical_protocol", "async_test_simulated_basic", AsyncSimulatedBasicTest,
    )
    unittest.main()
"""
