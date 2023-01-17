#!/usr/bin/python3
import random
import string
import struct

import rospy
from rospy_tutorials.srv import AddTwoInts
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from .application_packet import ApplicationPacket
from .board import USBtoCANBoard
from .utils import PayloadTooLargeException


class CANDeviceHandle:
    """
    Base class to allow developers to write handles for communication with a
    particular CAN device. The two methods of the handle allow the handle to listen
    to incoming data, as well as send data.

    .. code-block:: python

        class ExampleEchoDeviceHandle(CANDeviceHandle):
            def __init__(self, driver, device_id):
                self.last_sent = None
                self.send_new_string()
                super().__init__(driver, device_id)

            def on_data(self, data: bytes):
                if self.last_sent is not None and data == self.last_sent[0]:
                    print(
                        "SUCCESSFULLY echoed {} in {}seconds".format(
                            self.last_sent[0], (rospy.Time.now() - self.last_sent[1]).to_sec()
                        )
                    )
                    rospy.sleep(0.15)
                    self.send_new_string()

            def send_new_string(self):
                test = "".join([random.choice(string.ascii_letters) for _ in range(4)])
                self.last_sent = (test, rospy.Time.now())
                self.send_data(test.encode())
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
        del data

    def send_data(self, data: bytes, can_id: int = 0):
        """
        Sends data to the device.

        Args:
            data (bytes): The data payload to send to the device.
            can_id (int): The CAN device ID to send data to. Defaults to 0.

        Raises:
            PayloadTooLargeException: The payload is larger than 8 bytes.
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
        self.count = 0
        self._srv = rospy.Service("start_echo", Trigger, self.srv_req)

    def srv_req(self, req: TriggerRequest):
        while self.count < 10:
            if not self.send_new_string(4):
                return TriggerResponse(False, "Unable to send string of length four.")

        try:
            self.send_new_string(8)
        except PayloadTooLargeException:
            pass
        except:
            return TriggerResponse(False, "Testing large strings failed.")
        return TriggerResponse(True, "Complete!")

    def on_data(self, data):
        data = ApplicationPacket.from_bytes(data, expected_identifier=37).payload
        if self.last_sent is None:
            raise RuntimeError(f"Received {data} but have not yet sent anything")
        elif data.decode() != self.last_sent[0]:
            raise ValueError(
                f"ERROR! Received {data.decode()} but last sent {self.last_sent}"
            )
        else:
            self.count += 1

    def send_new_string(self, length: int = 4):
        # Example string to test with
        test = "".join([random.choice(string.ascii_letters) for _ in range(length)])
        self.last_sent = (test, rospy.Time.now())
        self.send_data(bytes(ApplicationPacket(37, test.encode())))
        start = rospy.Time.now()
        count_now = self.count
        while self.count == count_now:
            if rospy.Time.now() - start > rospy.Duration(1):
                return False
        return True


class ExampleAdderDeviceHandle(CANDeviceHandle):
    """
    An example implementation of a CANDeviceHandle which will handle
    a device that echos back any data sent to it.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.response_received = None
        self._srv = rospy.Service("add_two_ints", AddTwoInts, self.on_service_req)

    def on_service_req(self, req):
        payload = struct.pack("hh", req.a, req.b)
        self.response_received = None
        self.send_data(bytes(ApplicationPacket(37, payload)))
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
