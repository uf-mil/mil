import random
import string
from dataclasses import dataclass

import rospy
from rospy_tutorials.srv import AddTwoInts
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from .device import CANDeviceHandle, SimulatedCANDeviceHandle
from .packet import Packet


@dataclass
class ExampleEchoRequestPacket(
    Packet,
    msg_id=0x99,
    subclass_id=0x00,
    payload_format="10s",
):
    my_special_string: bytes


@dataclass
class ExampleEchoResponsePacket(
    Packet,
    msg_id=0x99,
    subclass_id=0x01,
    payload_format="10s",
):
    my_special_string: bytes


@dataclass
class ExampleAdderRequestPacket(
    Packet,
    msg_id=0x99,
    subclass_id=0x02,
    payload_format="BB",
):
    num_one: int
    num_two: int


@dataclass
class ExampleAdderResponsePacket(
    Packet,
    msg_id=0x99,
    subclass_id=0x03,
    payload_format="B",
):
    response: int


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
            if not self.send_new_string(10):
                return TriggerResponse(False, "Unable to send string of length ten.")
        return TriggerResponse(True, "Complete!")

    def on_data(self, data: ExampleEchoRequestPacket):
        response = data.my_special_string.decode()
        if self.last_sent is None:
            raise RuntimeError(f"Received {data} but have not yet sent anything")
        elif response != self.last_sent[0]:
            raise ValueError(
                f"ERROR! Received {response} but last sent {self.last_sent}",
            )
        else:
            self.count += 1

    def send_new_string(self, length: int = 10):
        # Example string to test with
        test = "".join([random.choice(string.ascii_letters) for _ in range(length)])
        self.last_sent = (test, rospy.Time.now())
        self.send_data(ExampleEchoRequestPacket(test.encode()))
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
        self.response_received = None
        self.send_data(ExampleAdderRequestPacket(req.a, req.b))
        start = rospy.Time.now()
        while self.response_received is None:
            if rospy.Time.now() - start > rospy.Duration(1):
                return -1
        return self.response_received.response

    def on_data(self, data: ExampleAdderResponsePacket):
        self.response_received = data


class ExampleSimulatedEchoDevice(SimulatedCANDeviceHandle):
    """
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmitted data in a buffer.
    When data is requested, it echos this data back.
    """

    def __init__(self, handle, inbound_packets):
        # Call parent classes constructor
        super().__init__(handle, inbound_packets)

    def on_data(self, data: ExampleEchoRequestPacket):
        # Echo data received back onto the bus
        self.send_data(bytes(ExampleEchoResponsePacket(data.my_special_string)))


class ExampleSimulatedAdderDevice(SimulatedCANDeviceHandle):
    """
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmitted data in a buffer.
    When data is requested, it echos this data back.
    """

    def __init__(self, handle, inbound_packets):
        # Call parent classes constructor
        super().__init__(handle, inbound_packets)

    def on_data(self, data: ExampleAdderRequestPacket):
        self.send_data(bytes(ExampleAdderResponsePacket(data.num_one + data.num_two)))
