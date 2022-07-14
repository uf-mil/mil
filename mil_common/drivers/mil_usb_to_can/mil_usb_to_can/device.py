#!/usr/bin/python3
import random
import string
import struct

import rospy
from rospy_tutorials.srv import AddTwoInts

from .application_packet import ApplicationPacket
from .board import USBtoCANBoard


class CANDeviceHandle:
    """
    Base class to allow developers to write handles for communication with a
    particular CAN device.

    For examples of child classes of this class, see the ``device.py`` file in the
    ``mil_usb_to_can`` package.
    """

    def __init__(self, driver: USBtoCANBoard, device_id: int):
        """
        Args:
                driver (USBtoCANBoard): Driver that is used to communicate with the board.
                device_id (int): The CAN ID of the device this class will handle. Not currently used.
        """
        self._driver = driver
        self._device_id = device_id

    def on_data(self, data: bytes):
        """
        Called when data is received from the device this handle is registered for.

        Args:
            data (bytes): The data received.
        """
        pass

    def send_data(self, data: bytes, can_id: int = 0):
        """
        Sends data to the device.

        Args:
            data (bytes): The data payload to send to the device.
            can_id (int): The CAN device ID to send data to. Defaults to 0.
        """
        return self._driver.send_data(data, can_id=can_id)


class ExampleEchoDeviceHandle(CANDeviceHandle):
    """
    An example implementation of a CANDeviceHandle which will handle
    a device that echos back any data sent to it.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.last_sent = None
        self.send_new_string()

    def on_data(self, data):
        if self.last_sent is None:
            print(f"Received {data} but have not yet sent anything")
        elif data != self.last_sent[0]:
            print(f"ERROR! Reveived {data} but last sent {self.last_sent}")
        else:
            print(
                "SUCCESSFULLY echoed {} in {}seconds".format(
                    self.last_sent[0], (rospy.Time.now() - self.last_sent[1]).to_sec()
                )
            )
            rospy.sleep(0.15)
            self.send_new_string()

    def send_new_string(self):
        # Example string to test with
        test = "".join([random.choice(string.ascii_letters) for i in range(4)])
        self.last_sent = (test, rospy.Time.now())
        print(f"SENDING {test}")
        self.send_data(test)


class ExampleAdderDeviceHandle(CANDeviceHandle):
    """
    An example implementation of a CANDeviceHandle which will handle
    a device that echos back any data sent to it.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.correct_response = 37
        self.response_received = None
        self._srv = rospy.Service("add_two_ints", AddTwoInts, self.on_service_req)

    def on_service_req(self, req):
        payload = struct.pack("hh", req.a, req.b)
        self.correct_response = req.a + req.b
        self.response_received = None
        self.send_data(ApplicationPacket(37, payload).to_bytes())
        start = rospy.Time.now()
        while self.response_received is None:
            if rospy.Time.now() - start > rospy.Duration(1):
                return -1
        res = ApplicationPacket.from_bytes(
            self.response_received, expected_identifier=37
        )
        my_sum = struct.unpack("i", res.payload)
        return my_sum

    def on_data(self, data):
        self.response_received = data
