#!/usr/bin/python3
import rospy
from mil_usb_to_can import CANDeviceHandle
from ros_alarms import AlarmBroadcaster, AlarmListener
from ros_alarms.msg import Alarm
from rospy.timer import TimerEvent
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sub8_msgs.msg import Thrust

from .packets import (
    KILL_SEND_ID,
    THRUST_SEND_ID,
    HeartbeatMessage,
    KillMessage,
    StatusMessage,
    ThrustPacket,
)
from .thruster import make_thruster_dictionary


class ThrusterAndKillBoard(CANDeviceHandle):
    """
    Device handle for the thrust and kill board.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Initialize thruster mapping from params
        self.thrusters = make_thruster_dictionary(
            rospy.get_param("/thruster_layout/thrusters")
        )
        # Tracks last hw-kill alarm update
        self._last_hw_kill = None
        # Tracks last go status broadcasted
        self._last_go = None
        # Used to raise/clear hw-kill when board updates
        self._kill_broadcaster = AlarmBroadcaster("hw-kill")
        # Alarm broadaster for GO command
        self._go_alarm_broadcaster = AlarmBroadcaster("go")
        # Listens to hw-kill updates to ensure another nodes doesn't manipulate it
        self._hw_kill_listener = AlarmListener(
            "hw-kill", callback_funct=self.on_hw_kill
        )
        # Provide service for alarm handler to set/clear the motherboard kill
        self._unkill_service = rospy.Service(
            "/set_mobo_kill", SetBool, self.set_mobo_kill
        )
        # Sends hearbeat to board
        self._hearbeat_timer = rospy.Timer(rospy.Duration(0.4), self.send_heartbeat)
        # Create a subscribe for thruster commands
        self._sub = rospy.Subscriber(
            "/thrusters/thrust", Thrust, self.on_command, queue_size=10
        )

    def set_mobo_kill(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Called on service calls to ``/set_mobo_kill``, sending the appropriate
        packet to the board to unassert or assert to motherboard-origin kill.

        Args:
            req (SetBoolRequest): The service request.

        Returns:
            SetBoolResponse: The service response.
        """
        self.send_data(
            KillMessage.create_kill_message(
                command=True, hard=False, asserted=req.data
            ).to_bytes(),
            can_id=KILL_SEND_ID,
        )
        return SetBoolResponse(success=True)

    def send_heartbeat(self, _: TimerEvent) -> None:
        """
        Send a special heartbeat packet. Called by a recurring timer set upon
        initialization.
        """
        self.send_data(HeartbeatMessage.create().to_bytes(), can_id=KILL_SEND_ID)

    def on_hw_kill(self, alarm: Alarm) -> None:
        """
        Update the classes' hw-kill alarm to the latest update.

        Args:
            alarm (:class:`~ros_alarms.msg._Alarm.Alarm`): The alarm message to update with.
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
                    "Command received for {}, but this is not a thruster.".format(
                        cmd.name
                    )
                )
                continue
            # Map commanded thrust (in newetons) to effort value (-1 to 1)
            effort = self.thrusters[cmd.name].effort_from_thrust(cmd.thrust)
            # Send packet to command specified thruster the specified force
            packet = ThrustPacket.create_thrust_packet(
                ThrustPacket.ID_MAPPING[cmd.name], effort
            )
            self.send_data(packet.to_bytes(), can_id=THRUST_SEND_ID)

    def update_hw_kill(self, status: StatusMessage) -> None:
        """
        If needed, update the hw-kill alarm so it reflects the latest status from the board.

        Args:
            status (StatusMessage): The status message.
        """
        # Set severity / problem message appropriately
        severity = 0
        message = ""
        if status.hard_killed:
            severity = 2
            message = "Hard kill"
        elif status.soft_killed is True:
            severity = 1
            reasons = []
            if status.switch_soft_kill:
                reasons.append("switch")
            if status.mobo_soft_kill:
                reasons.append("mobo")
            if status.heartbeat_lost:
                reasons.append("hearbeat")
            message = "Soft kill from: " + ",".join(reasons)
        raised = severity != 0

        # If the current status differs from the alarm, update the alarm
        if (
            self._last_hw_kill is None
            or self._last_hw_kill.raised != raised
            or self._last_hw_kill.problem_description != message
            or self._last_hw_kill.severity != severity
        ):
            if raised:
                self._kill_broadcaster.raise_alarm(
                    severity=severity, problem_description=message
                )
            else:
                self._kill_broadcaster.clear_alarm(severity=severity)

    def on_data(self, data: bytes) -> None:
        """
        Parse the two bytes and raise kills according to a set of specifications
        listed below.

        Specifications of the first byte (where "asserted" is 1 and "unasserted" is 0):

        +------------+-------------------------------------------+
        | First Byte | Meaning                                   |
        +============+===========================================+
        | Bit 7      | Hard kill status, changed by the Hall     |
        |            | Effect switch. If this becomes 1, the     |
        |            | shutdown procedure begins.                |
        +------------+-------------------------------------------+
        | Bit 6      | The generalized soft kill status. Becomes |
        |            | 1 if any of the specific soft kill flags  |
        |            | become 1. Returns to 0 if all kills are   |
        |            | cleared and the thrusters are done        |
        |            | re-initializing.                          |
        +------------+-------------------------------------------+
        | Bit 5      | Soft kill flag for the Hall Effect sensor |
        +------------+-------------------------------------------+
        | Bit 4      | Motherboard command soft kill flag.       |
        +------------+-------------------------------------------+
        | Bit 3      | Soft kill indicating heartbeat was lost.  |
        |            | Times out after one second.               |
        +------------+-------------------------------------------+
        | Bit 2      |                                           |
        | Bit 1      | Reserved flags.                           |
        | Bit 0      |                                           |
        +------------+-------------------------------------------+

        Specifications of the second byte (where "pressed" is 1 and "unpressed" is 0):

        +-------------+-------------------------------------------+
        | Second Byte | Meaning                                   |
        +=============+===========================================+
        | Bit 7       | Hard kill status representing the Hall    |
        |             | Effect switch. If this becomes 0 ("on"),  |
        |             | the hard kill in the previous byte        |
        |             | becomes 1.                                |
        +-------------+-------------------------------------------+
        | Bit 6       | The soft kill status of the Hall Effect   |
        |             | sensor. If this is "pressed" (1) then bit |
        |             | 5 of the previous byte becomes "unkilled" |
        |             | (0). Removing the magnet "unpresses" the  |
        |             | switch.                                   |
        +-------------+-------------------------------------------+
        | Bit 5       | Go Hall Effect switch status. "Pressed"   |
        |             | is 1, removing the magnet "unpresses" the |
        |             | switch.                                   |
        +-------------+-------------------------------------------+
        | Bit 4       | Thruster initializing status. Becomes 1   |
        |             | when the board is unkilling and starts    |
        |             | powering thrusters. After the "grace      |
        |             | period" it becomes 0 and the overall soft |
        |             | kill status becomes 0. The flag also      |
        |             | becomes 0 if killed in the middle of      |
        |             | initializing thrusters.                   |
        +-------------+-------------------------------------------+
        | Bit 3       |                                           |
        | Bit 2       |                                           |
        | Bit 1       | Reserved flags.                           |
        | Bit 0       |                                           |
        +-------------+-------------------------------------------+
        """
        status = StatusMessage.from_bytes(data)
        self.update_hw_kill(status)

        go = status.go_switch
        if self._last_go is None or go != self._last_go:
            if go:
                self._go_alarm_broadcaster.raise_alarm(
                    problem_description="Go plug pulled!"
                )
            else:
                self._go_alarm_broadcaster.clear_alarm(
                    problem_description="Go plug returned"
                )
            self._last_go = go
