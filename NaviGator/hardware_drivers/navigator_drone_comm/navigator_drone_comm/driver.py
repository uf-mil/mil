import threading
from typing import Union

import rospy
from electrical_protocol import ROSSerialDevice
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point
from navigator_msgs.msg import DroneTarget, DroneTin
from navigator_msgs.srv import (
    DroneMission,
    DroneMissionRequest,
    MessageUAVReplenishment,
    MessageUAVReplenishmentRequest,
    MessageUAVSearchReport,
    MessageUAVSearchReportRequest,
)
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

SendPackets = Union[HeartbeatSetPacket, EStopPacket, StartPacket, StopPacket]
RecvPackets = Union[HeartbeatReceivePacket, GPSDronePacket, TargetPacket, TinPacket]


class DroneCommDevice(
    ROSSerialDevice[SendPackets, RecvPackets],
):
    def __init__(self, port: str):
        super().__init__(port, 57600)
        self.gps_pub = rospy.Publisher("~gps", Point, queue_size=5)
        self.target_pub = rospy.Publisher(
            "~target",
            DroneTarget,
            queue_size=5,
        )  # TODO replace with service call
        self.target_msg = {}
        self.tin_pub = rospy.Publisher("~tin", DroneTin, queue_size=5)
        self.drone_heartbeat_pub = rospy.Publisher(
            "~drone_heartbeat",
            Int8,
            queue_size=10,
        )
        self.received_seq_num = 0
        self.num_logos = 0
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
        packet: RecvPackets,
    ):
        if isinstance(packet, HeartbeatReceivePacket):
            rospy.loginfo("status %s", packet.status)
            self.drone_heartbeat_event.set()
            hbt_msg = Int8(packet.status)
            # hbt_msg = Header(self.received_seq_num, rospy.Time.now(), "drone_heartbeat") # TODO: publish int
            # self.received_seq_num += 1
            self.drone_heartbeat_pub.publish(hbt_msg)
        elif isinstance(packet, GPSDronePacket):
            point_msg = Point(packet.lat, packet.lon, packet.alt)
            self.gps_pub.publish(point_msg)
        elif isinstance(packet, TargetPacket):
            rospy.loginfo(str(packet))
            self.target_msg[self.num_logos] = [
                packet.lat,
                packet.lon,
                packet.logo.value,
            ]
            self.num_logos += 1
            rospy.loginfo(str(self.target_msg))
            # self.target_pub.publish(target_msg)
            if self.num_logos > 1:
                drone_target_proxy = rospy.ServiceProxy(
                    "uav_search_report_message",
                    MessageUAVSearchReport,
                )
                drone_target_proxy.wait_for_service()
                try:
                    drone_msg = MessageUAVSearchReportRequest()
                    drone_msg.object1 = self.target_msg[0][2]
                    drone_msg.object2 = self.target_msg[1][2]
                    drone_msg.uav_status = 2
                    point1 = GeoPoint()
                    point1.latitude = self.target_msg[0][0]
                    point1.longitude = self.target_msg[0][1]
                    point1.altitude = 0.0
                    point2 = GeoPoint()
                    point2.latitude = self.target_msg[1][0]
                    point2.longitude = self.target_msg[1][1]
                    point2.altitude = 0.0
                    drone_msg.object1_location = point1
                    drone_msg.object2_location = point2
                    rospy.loginfo(str(drone_msg))
                    response = drone_target_proxy(drone_msg)
                    rospy.loginfo(response)
                except Exception:
                    pass
        elif isinstance(packet, TinPacket):
            rospy.wait_for_service("uav_replenishment_message")
            try:
                uav_replenish_msg = MessageUAVReplenishmentRequest()
                uav_replenish_msg.uav_status = 2
                uav_replenish_msg.item_status = packet.status
                uav_replenish_msg.item_color = packet.color.name[0]

                rospy.loginfo(str(uav_replenish_msg))
                drone_tin_proxy = rospy.ServiceProxy(
                    "uav_replenishment_message",
                    MessageUAVReplenishment,
                )
                response = drone_tin_proxy(uav_replenish_msg)
                rospy.loginfo(response)
            except Exception:
                pass
        else:
            rospy.logerr("Received unexpected packet type from drone")
        return
