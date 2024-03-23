#! /usr/bin/env python3
import unittest
from dataclasses import dataclass

import rostest
from mil_usb_to_can.sub9 import Packet
from mil_usb_to_can.sub9.packet import SYNC_CHAR_1, SYNC_CHAR_2


@dataclass
class TestPacket(Packet, msg_id=0x47, subclass_id=0x44, payload_format="?Hd"):
    example_bool: bool
    example_int: int
    example_float: float


class BasicApplicationPacketTest(unittest.IsolatedAsyncioTestCase):
    """
    Tests basic application packet functionality.
    """

    def test_simple_packet(self):
        packet = TestPacket(False, 42, 3.14)
        self.assertEqual(packet.msg_id, 0x47)
        self.assertEqual(packet.subclass_id, 0x44)
        self.assertEqual(packet.payload_format, "?Hd")
        self.assertEqual(packet.example_bool, False)
        self.assertEqual(packet.example_int, 42)
        self.assertEqual(packet.example_float, 3.14)

    def test_assembled_packet(self):
        packet = TestPacket(False, 42, 3.14)
        assembled = bytes(TestPacket(False, 42, 3.14))
        self.assertEqual(assembled[0], SYNC_CHAR_1)
        self.assertEqual(assembled[1], SYNC_CHAR_2)
        self.assertEqual(assembled[2], packet.msg_id)
        self.assertEqual(assembled[3], packet.subclass_id)

    def test_format(self):
        packet = TestPacket(False, 42, 3.14)
        self.assertEqual(
            TestPacket.from_bytes(TestPacket.__bytes__(packet)),
            packet,
        )

    def test_comparisons(self):
        packet = TestPacket(False, 42, 3.14)
        packet_two = TestPacket(False, 42, 3.14)
        self.assertEqual(packet, packet_two)
        with self.assertRaises(TypeError):
            packet < packet_two
        with self.assertRaises(TypeError):
            packet > packet_two

    def test_checksum(self):
        self.assertEqual(Packet._calculate_checksum(b"abcde"), 0xF0C8)
        self.assertEqual(Packet._calculate_checksum(b"abcdefgh"), 0x2706)
        self.assertEqual(
            Packet._calculate_checksum(b"abcdeabcdeabcdeabcdeabcde"),
            0xBAF4,
        )


if __name__ == "__main__":
    packet = TestPacket(False, 42, 3.14)
    rostest.rosrun(
        "mil_usb_to_can",
        "test_application_packets_sub9",
        BasicApplicationPacketTest,
    )
    unittest.main()
