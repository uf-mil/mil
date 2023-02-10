import datetime
from dataclasses import dataclass

import rich
import rospy
from mil_usb_to_can import CANDeviceHandle, Packet
from sub8_thrust_and_kill_board import HeartbeatPacket, KillSetPacket, ThrustSetPacket
from sub_actuator_board import ActuatorPollRequestPacket, ActuatorSetPacket

from .packet import hexify


@dataclass
class TestPacket(Packet, msg_id=0x02, subclass_id=0x02, payload_format="f"):
    msg: float


class ExampleTestDevice(CANDeviceHandle):
    """
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmitted data in a buffer.
    When data is requested, it echos this data back.
    """

    def __init__(self, handle):
        # Call parent classes constructor
        super().__init__(handle)
        self.index = 0
        self.types = [
            HeartbeatPacket(),
            ThrustSetPacket(1, 0.5),
            KillSetPacket(1),
            ActuatorSetPacket(1, 2),
            ActuatorPollRequestPacket(),
        ]
        self._timer = rospy.Timer(rospy.Duration(1), self.send)

    def send(self, _):
        # packet = TestPacket(random.random())
        packet = self.types[self.index]
        rich.print(
            f"[bold chartreuse1]{datetime.datetime.now()}: Sending:  {packet}, {hexify(bytes(packet))}"
        )
        self.send_data(packet)
        self.index += 1
        self.index %= len(self.types)

    def on_data(self, data):
        # Echo data received back onto the bus
        rich.print(
            f"[bold gold1]{datetime.datetime.now()}  Received: {data}, {hexify(bytes(data))}"
        )
