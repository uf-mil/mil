from __future__ import annotations

from typing import Union

from electrical_protocol import AckPacket, NackPacket, ROSSerialDevice
from ros_alarms import AlarmListener

from .packets import KillReceivePacket, KillSetPacket, SetMovementModePacket


class NaviGatorLightKillDevice(
    ROSSerialDevice[
        Union[KillSetPacket, SetMovementModePacket],
        Union[KillReceivePacket, AckPacket, NackPacket],
    ],
):

    def __init__(self, port: str):
        super().__init__(port, 115200)
        self._hw_kill_listener = AlarmListener("hw-kill", self._hw_kill_alarm_cb)
        self._kill_listener = AlarmListener("kill", self.kill_alarm_cb)

    def _hw_kill_listener(self, alarm):
        pass

    def on_packet_received(self, packet):
        pass
