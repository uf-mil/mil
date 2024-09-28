#! /usr/bin/env python3
import struct
import unittest
from dataclasses import dataclass

import rostest
from electrical_protocol import Packet
from electrical_protocol.packet import SYNC_CHAR_1, SYNC_CHAR_2


@dataclass
class TestPacket(Packet, class_id=0x47, subclass_id=0x44, payload_format=">?Hd"):
    example_bool: bool
    example_int: int
    example_float: float


@dataclass
class TestPacketTwo(Packet, class_id=0x47, subclass_id=0x45, payload_format=""):
    pass


class BasicApplicationPacketTest(unittest.IsolatedAsyncioTestCase):
    """
    Tests basic application packet functionality.
    """

    def test_simple_packet(self):
        packet = TestPacket(False, 42, 3.14)
        self.assertEqual(packet.class_id, 0x47)
        self.assertEqual(packet.subclass_id, 0x44)
        self.assertEqual(packet.payload_format, ">?Hd")
        self.assertEqual(packet.example_bool, False)
        self.assertEqual(packet.example_int, 42)
        self.assertEqual(packet.example_float, 3.14)
        # 8 for all packets
        # 1 for bool
        # 2 for int
        # 8 for float
        self.assertEqual(len(packet), 8 + 1 + 2 + 8)
        self.assertEqual(TestPacket._expected_len(), 8 + 1 + 2 + 8)

    def test_assembled_packet(self):
        packet = TestPacket(False, 42, 3.14)
        assembled = bytes(TestPacket(False, 42, 3.14))
        self.assertEqual(assembled[0], SYNC_CHAR_1)
        self.assertEqual(assembled[1], SYNC_CHAR_2)
        self.assertEqual(assembled[2], packet.class_id)
        self.assertEqual(assembled[3], packet.subclass_id)

    def test_format(self):
        packet = TestPacket(False, 42, 3.14)
        self.assertEqual(
            TestPacket.from_bytes(bytes(packet)),
            packet,
        )
        self.assertEqual(
            Packet.from_bytes(bytes(packet)),
            packet,
        )
        with self.assertRaises(RuntimeError):
            TestPacketTwo.from_bytes(bytes(packet))

    def test_comparisons(self):
        packet = TestPacket(False, 42, 3.14)
        packet_two = TestPacket(False, 42, 3.14)
        self.assertEqual(packet, packet_two)
        with self.assertRaises(TypeError):
            packet < packet_two
        with self.assertRaises(TypeError):
            packet > packet_two

    def _pack_checksum(self, byte_string: bytes) -> int:
        checksum = Packet._calculate_checksum(byte_string)
        return int.from_bytes(struct.pack("<BB", *checksum), byteorder="big")

    def test_checksum(self):
        self.assertEqual(self._pack_checksum(b"abcde"), 0xF0C8)
        self.assertEqual(self._pack_checksum(b"abcdefgh"), 0x2706)
        self.assertEqual(
            self._pack_checksum(b"abcdeabcdeabcdeabcdeabcde"),
            0xB4FA,
        )


if __name__ == "__main__":
    packet = TestPacket(False, 42, 3.14)
    rostest.rosrun(
        "mil_usb_to_can",
        "test_application_packets_sub9",
        BasicApplicationPacketTest,
    )
    unittest.main()
