#! /usr/bin/env python3
import random
import string
import struct
import unittest

import rostest
from mil_usb_to_can import ApplicationPacket, CommandPacket


class BasicApplicationPacketTest(unittest.IsolatedAsyncioTestCase):
    """
    Tests basic application packt functionality.
    """

    def test_incorrect_identifiers(self):
        packet = ApplicationPacket(256, b"test")
        with self.assertRaises(struct.error):
            bytes(packet)
        packet = ApplicationPacket(-1, b"test")
        with self.assertRaises(struct.error):
            bytes(packet)
        packet = ApplicationPacket("a", b"test")
        with self.assertRaises(struct.error):
            bytes(packet)

    def test_format(self):
        letter = random.choice(string.ascii_letters)
        packet = ApplicationPacket(37, letter.encode())
        self.assertEqual(bytes(packet), struct.pack("B1s", 37, letter.encode()))
        self.assertEqual(
            ApplicationPacket.from_bytes(bytes(packet), expected_identifier=37), packet
        )

    def test_assembled(self):
        letter = random.choice(string.ascii_letters)
        packet = ApplicationPacket(37, letter.encode())
        command_packet = bytes(CommandPacket(bytes(packet)))
        data = struct.pack(f"B{len(bytes(packet))}sB", 0xC0, bytes(packet), 0xC1)
        checksum = CommandPacket.calculate_checksum(data)
        header_byte = (checksum << 3) | data[1]
        data = data[:1] + chr(header_byte).encode() + data[2:]
        self.assertEqual(command_packet, data)


if __name__ == "__main__":
    rostest.rosrun(
        "mil_usb_to_can", "test_application_packets", BasicApplicationPacketTest
    )
    unittest.main()
