#!/usr/bin/python3
from __future__ import annotations

import rospy
from mil_misc_tools import rospy_to_datetime
from mil_usb_to_can.sub9 import AckPacket, CANDeviceHandle, NackPacket
from ros_alarms import AlarmBroadcaster, AlarmListener
from ros_alarms_msgs.msg import Alarm
from rospy.timer import TimerEvent
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from subjugator_msgs.msg import Thrust

from .packets import (
    HeartbeatReceivePacket,
    HeartbeatSetPacket,
    KillReceivePacket,
    KillSetPacket,
    KillStatus,
    ThrustSetPacket,
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
        # Sends heartbeat to board
        self._heartbeat_timer = rospy.Timer(rospy.Duration(0.4), self.send_heartbeat)
        # Create a subscribe for thruster commands
        self._sub = rospy.Subscriber(
            "/thrusters/thrust",
            Thrust,
            self.on_command,
            queue_size=10,
        )
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
        self.send_data(KillSetPacket(req.data, KillStatus.SOFTWARE_REQUESTED))
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

    def send_heartbeat(self, _: TimerEvent) -> None:
        """
        Send a special heartbeat packet. Called by a recurring timer set upon
        initialization.
        """
        self.send_data(HeartbeatSetPacket())

    def on_hw_kill(self, alarm: Alarm) -> None:
        """
        Update the classes' hw-kill alarm to the latest update.

        Args:
            alarm (:class:`~ros_alarms_msgs.msg._Alarm.Alarm`): The alarm message to update with.
        """
        self._last_hw_kill = alarm

    def on_command(self, msg: Thrust) -> None:
        """
        When a thrust command message is received from the Subscriber, send the appropriate packet
        to the board.

        Args:
            msg (Thrust): The thrust message.
        """
        for cmd in msg.thruster_commands:
            # If we don't have a mapping for this thruster, ignore it
            if cmd.name not in self.thrusters:
                rospy.logwarn(
                    f"Command received for {cmd.name}, but this is not a thruster.",
                )
                continue
            # Map commanded thrust (in newetons) to effort value (-1 to 1)
            effort = self.thrusters[cmd.name].effort_from_thrust(cmd.thrust)
            # Send packet to command specified thruster the specified force
            packet = ThrustSetPacket(self.ID_MAPPING[cmd.name], effort)
            self.send_data(packet)

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
        data: AckPacket | NackPacket | HeartbeatReceivePacket | KillReceivePacket,
    ) -> None:
        """
        Parse the two bytes and raise kills according to a set of specifications
        listed below.
        """
        if isinstance(data, (AckPacket, NackPacket)):
            self._last_packet = data
        elif isinstance(data, HeartbeatReceivePacket):
            self._last_heartbeat = rospy.Time.now()
        elif isinstance(data, KillReceivePacket):
            if data.set is False:
                self.update_software_kill(False, "")
            elif data.status is KillStatus.MOBO_HEARTBEAT_LOST:
                self.update_software_kill(
                    True,
                    "Thrust/kill board lost heartbeat from motherboard.",
                )
            elif data.status is KillStatus.BATTERY_LOW:
                self.update_software_kill(True, "Battery too low.")
            elif data.status is KillStatus.KILL_SWITCH:
                self.update_software_kill(True, "Kill switch was pulled!")
            elif data.status is KillStatus.BOARD_HEARTBEAT_LOST:
                dt = rospy_to_datetime(self._last_heartbeat)
                self.update_software_kill(
                    True,
                    f"Motherboard is no longer hearing heartbeat from thrust/kill board. Last heard from board at {dt}.",
                )
            elif data.status is KillStatus.SOFTWARE_REQUESTED:
                self.update_software_kill(
                    True,
                    "Software requested kill.",
                )
        else:
            raise ValueError(f"Not expecting packet of type {data.__class__.__name__}!")
