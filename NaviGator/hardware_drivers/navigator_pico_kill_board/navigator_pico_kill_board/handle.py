#!/usr/bin/python3
from __future__ import annotations

import rospy
from mil_usb_to_can.sub9 import CANDeviceHandle, NackPacket
from ros_alarms import AlarmBroadcaster, AlarmListener
from ros_alarms_msgs.msg import Alarm
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from .packets import (
    KillRecievePacket,
    KillSetPacket,
    KillStatus,
)
from .thruster import make_thruster_dictionary


class ThrusterAndKillBoard(CANDeviceHandle):
    """
    Device handle for the thrust and kill board.
    """

    ID_MAPPING = {
        "FLH": 0,
        "FRH": 1,
        "FLV": 2,
        "FRV": 3,
        "BLH": 4,
        "BRH": 5,
        "BLV": 6,
        "BRV": 7,
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Initialize thruster mapping from params
        self.thrusters = make_thruster_dictionary(
            rospy.get_param("/thruster_layout/thrusters"),
        )
        # Tracks last hw-kill alarm update
        self._last_hw_kill = None
        # Used to raise/clear hw-kill when board updates
        self._kill_broadcaster = AlarmBroadcaster("hw-kill")
        # Listens to hw-kill updates to ensure another nodes doesn't manipulate it
        self._hw_kill_listener = AlarmListener(
            "hw-kill",
            callback_funct=self.on_hw_kill,
        )
        # Provide service for alarm handler to set/clear the motherboard kill
        self._unkill_service = rospy.Service("/set_mobo_kill", SetBool, self.set_kill)
        self._last_heartbeat = rospy.Time.now()
        self._last_packet = None
        self._updated_kill = False

    def set_kill(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Called on service calls to ``/set_mobo_kill``, sending the appropriate
        packet to the board to unassert or assert to motherboard-origin kill.

        Args:
            req (SetBoolRequest): The service request.

        Returns:
            SetBoolResponse: The service response.
        """
        self.send_data(KillSetPacket(req.data, KillStatus.MOBO_REQUESTED))
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(1):
            if self._last_packet is not None:
                break

        if isinstance(self._last_packet, NackPacket):
            raise RuntimeError("Request not acknowledged.")

        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(1):
            if self._updated_kill:
                break

        if self._updated_kill:
            self._updated_kill = False
            return SetBoolResponse(success=True)
        else:
            return SetBoolResponse(
                success=False,
                message="No response from board after 1 second.",
            )

    def on_hw_kill(self, alarm: Alarm) -> None:
        """
        Update the classes' hw-kill alarm to the latest update.

        Args:
            alarm (:class:`~ros_alarms_msgs.msg._Alarm.Alarm`): The alarm message to update with.
        """
        self._last_hw_kill = alarm

    def update_software_kill(self, raised: bool, message: str):
        # If the current status differs from the alarm, update the alarm
        severity = 2 if raised else 0
        self._updated_kill = True
        self._hw_kill_listener.wait_for_server()
        if (
            self._last_hw_kill is None
            or self._last_hw_kill.raised != raised
            or self._last_hw_kill.problem_description != message
            or self._last_hw_kill.severity != severity
        ):
            if raised:
                self._kill_broadcaster.raise_alarm(
                    severity=severity,
                    problem_description=message,
                )
            else:
                self._kill_broadcaster.clear_alarm(severity=severity)

    def on_data(
        self,
        data: KillRecievePacket,
    ) -> None:
        """
        Parse the two bytes and raise kills according to a set of specifications
        listed below.
        """
        print(data)
