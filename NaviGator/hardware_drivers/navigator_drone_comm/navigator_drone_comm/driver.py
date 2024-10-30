import threading
from typing import Union

import rospy
from electrical_protocol import ROSSerialDevice
from navigator_msgs.srv import DroneMission, DroneMissionRequest
from std_srvs.srv import Empty, EmptyRequest

from navigator_drone_comm import (
    EStopPacket,
    GPSDronePacket,
    HeartbeatReceivePacket,
    HeartbeatSetPacket,
    StartPacket,
    StopPacket,
    TargetPacket,
)


class DroneCommDevice(
    ROSSerialDevice[
        Union[HeartbeatSetPacket, EStopPacket, StartPacket, StopPacket],
        Union[HeartbeatReceivePacket, GPSDronePacket, TargetPacket],
    ],
):
    def __init__(self, port: str):
        super().__init__(port, 57600)
        self.estop_service = rospy.Service("~estop", Empty, self.estop)
        self.start_service = rospy.Service("~start", DroneMission, self.start)
        self.stop_service = rospy.Service("~stop", Empty, self.stop)
        self.boat_heartbeat_timer = rospy.Timer(rospy.Duration(1), self.heartbeat_send)
        self.drone_heartbeat_event = threading.Event()
        self.drone_heartbeat_timer = rospy.Timer(
            rospy.Duration(1),
            self.heartbeat_check,
        )
        rospy.loginfo("DroneCommDevice initialized")

    def estop(self, _: EmptyRequest):
        self.estop_send()
        return {}

    def start(self, request: DroneMissionRequest):
        self.start_send(mission_name=request.mission)
        return {}

    def stop(self, _: EmptyRequest):
        self.stop_send()
        return {}

    def heartbeat_send(self, _):
        # rospy.loginfo("sending heartbeat")
        self.send_packet(HeartbeatSetPacket())

    def heartbeat_check(self, _):
        passed_check = self.drone_heartbeat_event.wait(1)
        if passed_check:
            self.drone_heartbeat_event.clear()
        else:
            # self.stop_send() # Uncomment to stop drone if no heartbeat is received
            rospy.logerr("No heartbeat received from drone")

    def estop_send(self):
        # rospy.loginfo("sending EStop")
        self.send_packet(EStopPacket())

    def start_send(self, mission_name: str):
        # rospy.loginfo(f"sending Start for mission: %s", mission_name)
        self.send_packet(StartPacket(name=mission_name))

    def stop_send(self):
        # rospy.loginfo("sending Stop")
        self.send_packet(StopPacket())

    def on_packet_received(
        self,
        packet: Union[HeartbeatReceivePacket, GPSDronePacket, TargetPacket],
    ):
        if isinstance(packet, HeartbeatReceivePacket):
            self.drone_heartbeat_event.set()
        elif isinstance(packet, GPSDronePacket):
            rospy.loginfo("Received GPS packet: %s", packet)
        elif isinstance(packet, TargetPacket):
            rospy.loginfo("Received Target packet: %s", packet)
        else:
            rospy.logerr("Received unexpected packet type")
        return
