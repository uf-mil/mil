import threading
from typing import Union

import rospy
from electrical_protocol import ROSSerialDevice
from geometry_msgs.msg import Point
from navigator_msgs.msg import DroneTarget, DroneTin
from navigator_msgs.srv import DroneMission, DroneMissionRequest
from std_msgs.msg import Int8
from std_srvs.srv import Empty, EmptyRequest

from .packets import (
    EStopPacket,
    GPSDronePacket,
    HeartbeatReceivePacket,
    HeartbeatSetPacket,
    StartPacket,
    StopPacket,
    TargetPacket,
    TinPacket,
)


class DroneCommDevice(
    ROSSerialDevice[
        Union[HeartbeatSetPacket, EStopPacket, StartPacket, StopPacket],
        Union[HeartbeatReceivePacket, GPSDronePacket, TargetPacket, TinPacket],
    ],
):
    def __init__(self, port: str):
        super().__init__(port, 57600)
        self.gps_pub = rospy.Publisher("~gps", Point, queue_size=5)
        self.target_pub = rospy.Publisher(
            "~target",
            DroneTarget,
            queue_size=5,
        )  # TODO replace with service call
        self.tin_pub = rospy.Publisher("~tin", DroneTin, queue_size=5)
        self.drone_heartbeat_pub = rospy.Publisher(
            "~drone_heartbeat",
            Int8,
            queue_size=10,
        )
        self.received_seq_num = 0
        self.estop_service = rospy.Service("~estop", Empty, self.estop)
        self.start_service = rospy.Service("~start", DroneMission, self.start)
        self.stop_service = rospy.Service("~stop", Empty, self.stop)
        # self.boat_heartbeat_timer = rospy.Timer(rospy.Duration(1), self.heartbeat_send)
        # self.heartbeat_service = self.nh.get_service_client(
        #     ""
        # )
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
        pass

    def heartbeat_check(self, _):
        passed_check = self.drone_heartbeat_event.wait(1)
        if passed_check:
            self.drone_heartbeat_event.clear()
        else:
            # self.stop_send() # Uncomment to stop drone if no heartbeat is received
            rospy.logerr("No heartbeat received from drone")
            pass

    def estop_send(self):
        self.send_packet(EStopPacket())

    def start_send(self, mission_name: str):
        self.send_packet(StartPacket(name=mission_name))

    def stop_send(self):
        self.send_packet(StopPacket())

    def on_packet_received(
        self,
        packet: Union[HeartbeatReceivePacket, GPSDronePacket, TargetPacket, TinPacket],
    ):
        if isinstance(packet, HeartbeatReceivePacket):
            rospy.loginfo(packet.status)
            self.drone_heartbeat_event.set()
            hbt_msg = Int8(packet.status)
            # hbt_msg = Header(self.received_seq_num, rospy.Time.now(), "drone_heartbeat") # TODO: publish int
            # self.received_seq_num += 1
            self.drone_heartbeat_pub.publish(hbt_msg)
        elif isinstance(packet, GPSDronePacket):
            point_msg = Point(packet.lat, packet.lon, packet.alt)
            self.gps_pub.publish(point_msg)
        elif isinstance(packet, TargetPacket):
            target_msg = DroneTarget(packet.lat, packet.lon, packet.logo.name)
            self.target_pub.publish(target_msg)
        elif isinstance(packet, TinPacket):
            tin_msg = DroneTin(packet.status, packet.color)
            self.tin_pub.publish(tin_msg)
        else:
            rospy.logerr("Received unexpected packet type from drone")
        return
