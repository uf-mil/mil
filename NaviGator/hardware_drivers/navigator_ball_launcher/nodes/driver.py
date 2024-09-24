#! /usr/bin/env python3
from __future__ import annotations

from typing import Union

import rospy
from electrical_protocol import AckPacket, NackPacket, ROSSerialDevice
from navigator_ball_launcher.packets import ReleaseBallPacket, SetSpinPacket
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest


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
        self.heard_ack = False
        self.heard_nack = False

    def drop_ball(self, _: EmptyRequest):
        rospy.loginfo("Dropping ball...")
        self.send_packet(ReleaseBallPacket())
        rospy.sleep(1)
        if self.heard_nack:
            rospy.logerr("Failed to drop ball (heard NACK from board)")
        elif not self.heard_ack:
            rospy.logerr("Failed to drop ball (no response from board)")
        return {}

    def spin(self, request: SetBoolRequest):
        if request.data:
            rospy.loginfo("Spinning up the ball launcher...")
        else:
            rospy.loginfo("Stopping the ball launcher...")
        self.send_packet(SetSpinPacket(request.data))
        rospy.sleep(1)
        if self.heard_nack:
            rospy.logerr("Failed to set spin (heard NACK from board)")
        elif not self.heard_ack:
            rospy.logerr("Failed to set spin (no response from board)")
        return {}

    def on_packet_received(self, packet: AckPacket | NackPacket) -> None:
        if isinstance(packet, AckPacket):
            self.heard_ack = True
        elif isinstance(packet, NackPacket):
            self.heard_nack = True


if __name__ == "__main__":
    rospy.init_node("ball_launcher")
    device = BallLauncherDevice(str(rospy.get_param("~port")))
    rospy.spin()
