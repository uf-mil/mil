#!/usr/bin/env python3
import rospy
from sub8_actuator_board.srv import SetValve, SetValveRequest, SetValveResponse
from mil_usb_to_can import CANDeviceHandle
from .packets import CommandMessage, InvalidAddressException, FeedbackMessage, SEND_ID


__author__ = "John Morin"


class ActuatorBoard(CANDeviceHandle):
    """
    Device handle for the actuator board. Because this class implements a CAN device,
    it inherits from the :class:`CANDeviceHandle` class.
    """
    def __init__(self, *args, **kwargs):
        super(ActuatorBoard, self).__init__(*args, **kwargs)
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
        self.send_data(message.to_bytes(), can_id=SEND_ID)
        rospy.loginfo(
            "Set valve {} {}".format(req.actuator, "opened" if req.opened else "closed")
        )
        # Wait some time for board to process command
        rospy.sleep(0.01)
        # Request the status of the valve just commanded to ensure it worked
        self.send_data(
            CommandMessage.create_command_message(
                address=req.actuator, write=False
            ).to_bytes(),
            can_id=SEND_ID,
        )
        return {"success": True}

    def on_data(self, data) -> None:
        """
        Process data received from board.
        """
        # Ensure packet contains correct identifier byte
        if FeedbackMessage.IDENTIFIER != ord(data[0]):
            rospy.logwarn(
                "Received packet with wrong identifer byte {}".format(ord(data[0]))
            )
            return
        # Parse message and (for now) just log it
        message = FeedbackMessage.from_bytes(data)
        rospy.loginfo("ActuatorBoard received {}".format(message))
