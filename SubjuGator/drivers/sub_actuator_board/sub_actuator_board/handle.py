#!/usr/bin/env python3
from __future__ import annotations

import rospy
from mil_usb_to_can.sub9 import AckPacket, CANDeviceHandle

from sub_actuator_board.srv import (
    GetValve,
    GetValveRequest,
    GetValveResponse,
    SetValve,
    SetValveRequest,
)

from .packets import (
    ActuatorPollRequestPacket,
    ActuatorPollResponsePacket,
    ActuatorSetPacket,
)


class ActuatorBoard(CANDeviceHandle):
    """
    Device handle for the actuator board. Because this class implements a CAN device,
    it inherits from the :class:`CANDeviceHandle` class.
    """

    _recent_response: ActuatorPollResponsePacket | None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._set_service = rospy.Service("/set_valve", SetValve, self.set_valve)
        self._get_service = rospy.Service("/get_valve", GetValve, self.get_valve)
        self._recent_response = None

    def set_valve(self, req: SetValveRequest) -> dict:
        """
        Called when the ``/set_valve`` service is requested. Creates a message to
        control the valve and sends it through the inherited device handle.

        Args:
            req (SetValveRequest): Request to set the valve.

        Returns:
            dict: List of information which is casted into a SetValveResponse.
        """
        # Send board command to open or close specified valve
        message = ActuatorSetPacket(address=req.actuator, open=req.opened)
        self.send_data(message)
        rospy.loginfo(
            "Set valve {} {}".format(
                req.actuator,
                "opened" if req.opened else "closed",
            ),
        )
        # Wait some time for board to process command
        rospy.sleep(0.01)
        # Request the status of the valve just commanded to ensure it worked
        self.send_data(ActuatorPollRequestPacket())
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(2):
            if self._recent_response is not None:
                break

        success = False
        if self._recent_response is not None:
            if not req.opened:
                success = not (self._recent_response.values & (1 << req.actuator))
            else:
                success = self._recent_response.values & (1 << req.actuator)
        response = {"success": success}
        self._recent_response = None
        return response

    def get_valve(self, req: GetValveRequest) -> GetValveResponse:
        message = ActuatorPollRequestPacket()
        self.send_data(message)
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(10):
            if self._recent_response is not None:
                break

        if not self._recent_response:
            raise RuntimeError("No response from the board within 10 seconds.")

        response = GetValveResponse(
            opened=self._recent_response.values & (1 << req.actuator),
        )
        self._recent_response = None
        return response

    def on_data(self, packet: ActuatorPollResponsePacket | AckPacket) -> None:
        """
        Process data received from board.
        """
        if isinstance(packet, ActuatorPollResponsePacket):
            self._recent_response = packet
