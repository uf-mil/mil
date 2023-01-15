#!/usr/bin/python3
import rospy
from mil_usb_to_can import SimulatedCANDevice
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from .packets import (
    KILL_SEND_ID,
    THRUST_SEND_ID,
    HeartbeatMessage,
    KillMessage,
    StatusMessage,
    ThrustPacket,
)


class ThrusterAndKillBoardSimulation(SimulatedCANDevice):
    """
    Serial simulator for the thruster and kill board,
    providing services to simulate physical plug connections/disconnections.

    Inherits from :class:`~mil_usb_to_can.SimulatedCANDevice`.

    Attributes:
        hard_kill_plug_pulled (bool): Whether the hard kill was set.
        hard_kill_mobo (bool): Whether the motherboard experienced a hard kill request.
        soft_kill_plug_pulled (bool): Whether the soft kill was set.
        soft_kill_mobo (bool): Whether the motherboard experienced a soft kill request.
    """

    HEARTBEAT_TIMEOUT_SECONDS = rospy.Duration(1.0)

    def __init__(self, *args, **kwargs):
        self.hard_kill_plug_pulled = False
        self.hard_kill_mobo = False
        self.soft_kill_plug_pulled = False
        self.soft_kill_mobo = False
        self.go_button = False
        self._last_heartbeat = None
        super().__init__(*args, **kwargs)
        self._update_timer = rospy.Timer(rospy.Duration(1), self.send_updates)
        self._soft_kill = rospy.Service(
            "/simulate_soft_kill", SetBool, self.set_soft_kill
        )
        self._hard_kill = rospy.Service(
            "/simulate_hard_kill", SetBool, self.set_hard_kill
        )
        self._go_srv = rospy.Service("/simulate_go", SetBool, self._on_go_srv)

    def _on_go_srv(self, req):
        self.go_button = req.data
        return {"success": True}

    def set_soft_kill(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Called by the `/simulate_soft_kill` service to set the soft kill state of the
        simluated device.

        Args:
            req (SetBoolRequest): The request to set the service with.

        Returns:
            SetBoolResponse: The response to the service that the operation was successful.
        """
        self.soft_kill_plug_pulled = req.data
        return SetBoolResponse(success=True)

    def set_hard_kill(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Called by the `/simulate_hard_kill` service to set the hard kill state of the
        simluated device.

        Args:
            req (SetBoolRequest): The request to set the service with.

        Returns:
            SetBoolResponse: The response to the service that the operation was successful.
        """
        self.hard_kill_plug_pulled = req.data
        return SetBoolResponse(success=True)

    @property
    def hard_killed(self) -> bool:
        """
        Whether the board was hard killed.

        Returns:
            bool: The status of the board being hard killed.
        """
        return self.hard_kill_mobo or self.hard_kill_plug_pulled

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

    @property
    def soft_killed(self) -> bool:
        """
        Whether the board was soft killed.

        Returns:
            bool: The status of the board being soft killed.
        """
        return (
            self.soft_kill_plug_pulled or self.soft_kill_mobo or self.heartbeat_timedout
        )

    def send_updates(self, *args) -> None:
        """
        Sends data about the class in a new status message.
        """
        status = StatusMessage(
            self.heartbeat_timedout,
            self.soft_kill_mobo,
            self.soft_kill_plug_pulled,
            self.soft_killed,
            self.hard_killed,
            False,
            self.go_button,
            not self.soft_kill_plug_pulled,
            not self.hard_kill_plug_pulled,
        )
        self.send_data(bytes(status))

    def on_data(self, data: bytes, can_id: int) -> None:
        """
        Serves as the data handler for the device. Handles :class:`KillMessage`,
        :class:`ThrustPacket`, and :class:`HeartbeatMessage` types.
        """
        assert can_id == THRUST_SEND_ID or can_id == KILL_SEND_ID
        if KillMessage.IDENTIFIER == data[0]:
            packet = KillMessage.from_bytes(data)
            assert packet.is_command
            assert packet.is_hard or packet.is_soft
            if packet.is_hard:
                self.hard_kill_mobo = packet.is_asserted
            elif packet.is_soft:
                self.soft_kill_mobo = packet.is_asserted
            self.send_updates()
        elif ThrustPacket.IDENTIFIER == data[0]:
            packet = ThrustPacket.from_bytes(data)
        elif HeartbeatMessage.IDENTIFIER == data[0]:
            packet = HeartbeatMessage.from_bytes(data)
            self._last_heartbeat = rospy.Time.now()
        else:
            assert False, "No recognized identifier"
