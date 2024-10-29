from __future__ import annotations

from typing import Union

import rospy
from electrical_protocol import AckPacket, NackPacket, ROSSerialDevice
from ros_alarms import AlarmBroadcaster, AlarmListener
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from .packets import (
    KillReceivePacket,
    KillSetPacket,
    KillStatus,
    SetMovementModePacket,
    SuspendHeartbeatPacket,
)

SendPackets = Union[KillSetPacket, SetMovementModePacket, SuspendHeartbeatPacket]
RecvPackets = Union[KillReceivePacket, AckPacket, NackPacket]


class KillLightBoardDevice(
    ROSSerialDevice[
        SendPackets,
        RecvPackets,
    ],
):

    ALARM = "hw-kill"  # Alarm to raise when hardware kill is detected
    REMOTE_CONTROL_WRENCH_STATES = [
        "rc",
        "/wrench/rc",
        "keyboard",
        "/wrench/keyboard",
    ]  # Wrenches which activate YELLOW LED (rc)
    AUTONOMOUS_WRENCH_STATES = [
        "autonomous",
        "/wrench/autonomous",
    ]  # Wrenches which activate GREEN LED

    _prev_wrench: None | str
    _ack_heard: rospy.Time | None
    _nack_heard: rospy.Time | None

    def __init__(self, port: str):
        super().__init__(port, 115200)
        self._hw_killed = False
        self._hw_kill_broadcaster = AlarmBroadcaster("hw-kill")
        self._hw_kill_broadcaster.wait_for_server()

        self.kill_broadcaster = AlarmBroadcaster("kill")
        self.kill_broadcaster.wait_for_server()

        self._network_kill_listener = AlarmListener(
            "network-loss",
            self._network_kill_alarm_cb,
        )
        self._network_kill_listener.wait_for_server()

        self._hw_kill_listener = AlarmListener("hw-kill", self._hw_kill_alarm_cb)
        self._hw_kill_listener.wait_for_server()

        self._kill_listener = AlarmListener("kill", self._kill_alarm_cb)
        self._kill_listener.wait_for_server()

        self._wrench_sub = rospy.Subscriber("/wrench/selected", String, self._wrench_cb)
        self._prev_wrench = None

        # Ignore missing heartbeat messages service
        self._ignore_heartbeat_srv = rospy.Service(
            "ignore_heartbeat",
            SetBool,
            self._ignore_heartbeat,
        )

    def _wrench_cb(self, wrench: String):
        if self._prev_wrench != wrench.data:
            if wrench.data in self.REMOTE_CONTROL_WRENCH_STATES:
                self.send_packet(SetMovementModePacket(False))
            elif wrench.data in self.AUTONOMOUS_WRENCH_STATES:
                self.send_packet(SetMovementModePacket(True))
        self._prev_wrench = wrench.data

    def _ignore_heartbeat(self, data: SetBoolRequest):
        response = None
        if data.data:
            response = "Instructing RF Kill system to ignore missing heartbeat packets."
            rospy.logwarn(response)
        else:
            response = "RF Kill system monitoring missing heartbeat packets once again."
            rospy.loginfo(response)
        self.send_packet(SuspendHeartbeatPacket(data.data))
        return SetBoolResponse(True, response)

    def _network_kill_alarm_cb(self, alarm):
        """
        If the network loss alarm is raised, we want to trigger a software kill
        """
        if alarm.raised and not self._hw_killed:
            self.send_raise()
        elif not alarm.raised and self._hw_killed:
            self.request_lower()

    def _hw_kill_alarm_cb(self, alarm):
        """
        If the user raises the hw-kill alarm, we want to trigger a hardware kill
        """
        if alarm.raised and not self._hw_killed:
            self.send_raise()
        elif not alarm.raised and self._hw_killed:
            self.request_lower()

    def _kill_alarm_cb(self, alarm):
        pass

    def send_raise(self):
        self.send_packet(KillSetPacket(True, KillStatus.MOBO_REQUESTED))

    def request_lower(self):
        self.send_packet(KillSetPacket(False, KillStatus.MOBO_REQUESTED))

    def raise_hw_alarm(self):
        self._hw_killed = True
        self._hw_kill_broadcaster.raise_alarm()

    def lower_hw_alarm(self):
        self._hw_killed = False
        self._hw_kill_broadcaster.clear_alarm()

    def on_packet_received(self, packet: RecvPackets):
        # Kill is requested from the board itself
        if isinstance(packet, KillReceivePacket):
            if packet.set:
                self.raise_hw_alarm()
            else:
                self.lower_hw_alarm()
        # ACK packet --> Valid response to a command we sent
        elif isinstance(packet, (AckPacket, NackPacket)):
            print(f"received {packet}!")
