#!/usr/bin/python3
from mil_usb_to_can import AckPacket, SimulatedCANDeviceHandle

from .packets import ActuatorPollRequest, ActuatorPollResponse, ActuatorSetPacket


class ActuatorBoardSimulation(SimulatedCANDeviceHandle):
    """
    Simulator for the communication of the actuator board.

    Attributes:
        status (Dict[int, bool]): The status of the valves. The keys are each of the valve IDs,
          and the values are the statues of whether the valves are open.
    """

    def __init__(self, *args, **kwargs):
        # Tracks the status of the 12 valves
        self.status = {i: False for i in range(4)}
        super().__init__(*args, **kwargs)

    def on_data(self, packet: ActuatorSetPacket | ActuatorPollRequest) -> None:
        """
        Processes data received from motherboard / other devices. For each message received,
        the class' status attribute is updated if the message is asking to write, otherwise
        a feedback message is constructed and sent back.
        """
        # If message is writing a valve, store this change in the internal dictionary
        if isinstance(packet, ActuatorSetPacket):
            self.status[packet.address] = packet.open
            self.send_data(bytes(AckPacket()))

        # If message is a status request, send motherboard the status of the requested valve
        elif isinstance(packet, ActuatorPollRequest):
            self.send_data(
                bytes(
                    ActuatorPollResponse(
                        int("".join(str(x) for x in self.status), base=2)
                    )
                )
            )
