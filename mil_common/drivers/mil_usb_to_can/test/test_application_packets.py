#! /usr/bin/env python3
import random
import string
import struct
import unittest

import rostest
from mil_usb_to_can import ApplicationPacket


class BasicApplicationPacketTest(unittest.IsolatedAsyncioTestCase):
    """
    Tests basic application packt functionality.
    """

    def test_incorrect_identifiers(self):
        packet = ApplicationPacket(256, b"test")
        with self.assertRaises(struct.error):
            packet.to_bytes()
        packet = ApplicationPacket(-1, b"test")
        with self.assertRaises(struct.error):
            packet.to_bytes()
        packet = ApplicationPacket("a", b"test")
        with self.assertRaises(struct.error):
            packet.to_bytes()

    def test_format(self):
        letter = random.choice(string.ascii_letters)
        packet = ApplicationPacket(37, letter.encode())
        self.assertEqual(packet.to_bytes(), struct.pack("B1s", 37, letter.encode()))
        self.assertEqual(
            packet.from_bytes(packet.to_bytes(), expected_identifier=37), packet
        )


if __name__ == "__main__":
    rostest.rosrun(
        "mil_usb_to_can", "test_application_packets", BasicApplicationPacketTest
    )
    unittest.main()
