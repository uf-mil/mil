#!/usr/bin/python3
from __future__ import annotations

import rospy
from mil_usb_to_can import AckPacket, NackPacket, SimulatedCANDeviceHandle
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from .packets import (
    HeartbeatReceivePacket,
    HeartbeatSetPacket,
    KillReceivePacket,
    KillSetPacket,
    KillStatus,
    ThrustSetPacket,
)


class ThrusterAndKillBoardSimulation(SimulatedCANDeviceHandle):
    """
    Serial simulator for the thruster and kill board,
    providing services to simulate physical plug connections/disconnections.

    Inherits from :class:`~mil_usb_to_can.SimulatedCANDevice`.

    Attributes:
        kill (bool): Whether the hard kill was set.
    """

    HEARTBEAT_TIMEOUT_SECONDS = rospy.Duration(1.0)

    def __init__(self, *args, **kwargs):
        self.kill = False
        self._last_heartbeat = None
        super().__init__(*args, **kwargs)
        self._update_timer = rospy.Timer(rospy.Duration(1), self._check_for_timeout)
        self._kill_srv = rospy.Service("/simulate_kill", SetBool, self.set_kill)

    def _check_for_timeout(self, _: rospy.timer.TimerEvent):
        if self.heartbeat_timedout and not self.kill:
            self.send_data(bytes(KillSetPacket(True, KillStatus.BOARD_HEARTBEAT_LOST)))
            self.kill = True

    def set_kill(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Called by the `/simulate_kill` service to set the kill state of the
        simluated device.

        Args:
            req (SetBoolRequest): The request to set the service with.

        Returns:
            SetBoolResponse: The response to the service that the operation was successful.
        """
        self.kill = req.data
        return SetBoolResponse(success=True)

    @property
    def heartbeat_timedout(self) -> bool:
        """
        Whether the heartbeat timed out.

        Returns:
            bool: The status of the heartbeat timing out.
        """
        return (
            self._last_heartbeat is None
            or (rospy.Time.now() - self._last_heartbeat)
            > self.HEARTBEAT_TIMEOUT_SECONDS
        )

    def on_data(
        self, packet: HeartbeatSetPacket | ThrustSetPacket | KillSetPacket
    ) -> None:
        """
        Serves as the data handler for the device. Handles :class:`KillMessage`,
        :class:`ThrustPacket`, and :class:`HeartbeatMessage` types.
        """
        if isinstance(packet, HeartbeatSetPacket):
            self._last_heartbeat = rospy.Time.now()
            self.send_data(bytes(HeartbeatReceivePacket()))

        elif isinstance(packet, ThrustSetPacket):
            self.send_data(bytes(AckPacket()))

        elif isinstance(packet, KillSetPacket):
            self.kill = packet.set
            self.send_data(bytes(AckPacket()))
            self.send_data(bytes(KillReceivePacket(packet.set, packet.status)))

        else:
            self.send_data(bytes(NackPacket()))
