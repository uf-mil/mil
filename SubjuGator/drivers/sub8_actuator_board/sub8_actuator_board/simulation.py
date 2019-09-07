#!/usr/bin/python
from mil_usb_to_can import SimulatedCANDevice
from packets import CommandMessage, FeedbackMessage, SEND_ID


class ActuatorBoardSimulation(SimulatedCANDevice):
    '''
    Simulator for the communication of the actuator board
    '''
    def __init__(self, *args, **kwargs):
        # Tracks the status of the 12 valves
        self.status = dict((i, False) for i in range(12))
        super(ActuatorBoardSimulation, self).__init__(*args, **kwargs)

    def on_data(self, data, can_id):
        '''
        Processes data received from motherboard / other devices
        '''
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
            response = FeedbackMessage.create_feedback_message(address=message.address, on=self.status[message.address])
            self.send_data(response.to_bytes())
