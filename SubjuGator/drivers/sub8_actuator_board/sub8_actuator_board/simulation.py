#!/usr/bin/python3
from mil_usb_to_can import SimulatedCANDevice

from .packets import SEND_ID, CommandMessage, FeedbackMessage


class ActuatorBoardSimulation(SimulatedCANDevice):
    """
    Simulator for the communication of the actuator board.

    Attributes:
        status (Dict[int, bool]): The status of the valves. The keys are each of the valve IDs,
          and the values are the statues of whether the valves are open.
    """

    def __init__(self, *args, **kwargs):
        # Tracks the status of the 12 valves
        self.status = {i: False for i in range(12)}
        super().__init__(*args, **kwargs)

    def on_data(self, data, can_id) -> None:
        """
        Processes data received from motherboard / other devices. For each message received,
        the class' status attribute is updated if the message is asking to write, otherwise
        a feedback message is constructed and sent back.
        """
        # If from wrong ID, ignore
        if can_id != SEND_ID:
            return

        # First byte should be the identifier char for the command message
        assert CommandMessage.IDENTIFIER == ord(data[0])

        # Parse message
        message = CommandMessage.from_bytes(data)

        # If message is writing a valve, store this change in the internal dictionary
        if message.write:
            self.status[message.address] = message.on

        # If message is a status request, send motherboard the status of the requested valve
        else:
            response = FeedbackMessage.create_feedback_message(
                address=message.address, on=self.status[message.address]
            )
            self.send_data(bytes(response))
