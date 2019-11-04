#!/usr/bin/python
import rospy
from srv import SetValve
from mil_usb_to_can import CANDeviceHandle
from packets import CommandMessage, InvalidAddressException, FeedbackMessage, SEND_ID


class ActuatorBoard(CANDeviceHandle):
    '''
    Device handle for the actuator board developed by John Morin
    '''
    def __init__(self, *args, **kwargs):
        super(ActuatorBoard, self).__init__(*args, **kwargs)
        self._service = rospy.Service('/set_valve', SetValve, self.set_valve)

    def set_valve(self, req):
        '''
        Called on service request to set valve
        '''
        # Send board command to open or close specified valve
        try:
            message = CommandMessage.create_command_message(address=req.actuator, write=True, on=req.opened)
        except InvalidAddressException as e:
            return {'success': False, 'message': str(e)}
        self.send_data(message.to_bytes(), can_id=SEND_ID)
        rospy.loginfo('Set valve {} {}'.format(req.actuator, "opened" if req.opened else "closed"))
        # Wait some time for board to process command
        rospy.sleep(0.01)
        # Request the status of the valve just commanded to ensure it worked
        self.send_data(CommandMessage.create_command_message(address=req.actuator, write=False).to_bytes(),
                       can_id=SEND_ID)
        return {'success': True}

    def on_data(self, data):
        '''
        Process data received from board
        '''
        # Ensure packet contains correct identifier byte
        if FeedbackMessage.IDENTIFIER != ord(data[0]):
            rospy.logwarn('Received packet with wrong identifer byte {}'.format(ord(data[0])))
            return
        # Parse message and (for now) just log it
        message = FeedbackMessage.from_bytes(data)
        rospy.loginfo('ActuatorBoard received {}'.format(message))
