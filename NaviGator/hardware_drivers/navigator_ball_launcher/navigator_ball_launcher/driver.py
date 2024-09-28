from __future__ import annotations

from typing import Union

import rospy
from electrical_protocol import AckPacket, NackPacket, ROSSerialDevice
from navigator_msgs.srv import BallLauncherDrops, BallLauncherDropsResponse
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest

from .packets import ReleaseBallPacket, SetSpinPacket


class BallLauncherDevice(
    ROSSerialDevice[
        Union[SetSpinPacket, ReleaseBallPacket],
        Union[AckPacket, NackPacket],
    ],
):

    heard_ack: bool
    heard_nack: bool

    def __init__(self, port: str):
        super().__init__(port, 115200)
        self.drop_service = rospy.Service("~drop_ball", Empty, self.drop_ball)
        self.spin_service = rospy.Service("~spin", SetBool, self.spin)
        self.drops_service = rospy.Service(
            "~number_of_drops",
            BallLauncherDrops,
            self._number_of_drops_srv,
        )
        self.heard_ack = False
        self.heard_nack = False
        self.drops = 0
        self.temp_timer = None
        print("done init")

    def get_drop_count(self) -> int:
        return self.drops

    def _number_of_drops_srv(self, _):
        return BallLauncherDropsResponse(self.get_drop_count())

    def _reset_ack(self):
        self.heard_ack = False
        self.heard_nack = False

    def _check_for_valid_response(self, event: str):
        if self.heard_nack:
            rospy.logerr(f"Failed to {event} (heard NACK from board)")
        elif not self.heard_ack:
            rospy.logerr(f"Failed to {event} (no response from board)")

    def _check_for_dropped_ball(self):
        self._check_for_valid_response("drop ball")

    def _check_for_spun(self):
        self._check_for_valid_response("set spin")

    def drop_ball(self, _: EmptyRequest):
        self._reset_ack()
        rospy.loginfo("Dropping ball...")
        self.drops += 1
        self.send_packet(ReleaseBallPacket())
        self.temp_timer = rospy.Timer(
            rospy.Duration(1),
            self._check_for_dropped_ball,
            oneshot=True,
        )
        return {}

    def spin(self, request: SetBoolRequest):
        self._reset_ack()
        if request.data:
            rospy.loginfo("Spinning up the ball launcher...")
        else:
            rospy.loginfo("Stopping the ball launcher...")
        self.send_packet(SetSpinPacket(request.data))
        self.temp_timer = rospy.Timer(
            rospy.Duration(1),
            self._check_for_spun,
            oneshot=True,
        )
        return {}

    def on_packet_received(self, packet: AckPacket | NackPacket) -> None:
        print("inc packet", packet)
        if isinstance(packet, AckPacket):
            self.heard_ack = True
        elif isinstance(packet, NackPacket):
            self.heard_nack = True
