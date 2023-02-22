#!/usr/bin/env python3
import rospy
from mil_usb_to_can import CANDeviceHandle
from sub8_actuator_board.srv import SetValve, SetValveRequest

from .packets import SEND_ID, CommandMessage, FeedbackMessage, InvalidAddressException

__author__ = "John Morin"


class ActuatorBoard(CANDeviceHandle):
    """
    Device handle for the actuator board. Because this class implements a CAN device,
    it inherits from the :class:`CANDeviceHandle` class.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._service = rospy.Service("/set_valve", SetValve, self.set_valve)

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
        try:
            message = CommandMessage.create_command_message(
                address=req.actuator, write=True, on=req.opened
            )
        except InvalidAddressException as e:
            return {"success": False, "message": str(e)}
        self.send_data(bytes(message), can_id=SEND_ID)
        rospy.loginfo(
            "Set valve {} {}".format(req.actuator, "opened" if req.opened else "closed")
        )
        # Wait some time for board to process command
        rospy.sleep(0.01)
        # Request the status of the valve just commanded to ensure it worked
        self.send_data(
            bytes(
                CommandMessage.create_command_message(address=req.actuator, write=False)
            ),
            can_id=SEND_ID,
        )
        return {"success": True}

    def on_data(self, data) -> None:
        """
        Process data received from board.
        """
        # Ensure packet contains correct identifier byte
        if FeedbackMessage.IDENTIFIER != ord(data[0]):
            rospy.logwarn(f"Received packet with wrong identifier byte {ord(data[0])}")
            return
        # Parse message and (for now) just log it
        message = FeedbackMessage.from_bytes(data)
        rospy.loginfo(f"ActuatorBoard received {message}")
