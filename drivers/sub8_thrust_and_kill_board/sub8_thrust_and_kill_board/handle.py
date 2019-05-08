#!/usr/bin/python
import rospy
from mil_usb_to_can import CANDeviceHandle
from packets import ThrustPacket, GoMessage, KillMessage, HeartbeatMessage, THRUST_SEND_ID, KILL_SEND_ID
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from ros_alarms import AlarmBroadcaster, AlarmListener


class ThrusterAndKillBoard(CANDeviceHandle):
    '''
    Device handle for the thrust and kill board.
    '''
    def __init__(self, *args, **kwargs):
        super(ThrusterAndKillBoard, self).__init__(*args, **kwargs)
        # Tracks last hw-kill alarm update
        self._last_hw_kill = None
        # Tracks last soft kill status received from board
        self._last_soft_kill = None
        # Tracks last hard kill status received from board
        self._last_hard_kill = None
        # Used to raise/clear hw-kill when board updates
        self._kill_broadcaster = AlarmBroadcaster('hw-kill')
        # Listens to hw-kill updates to ensure another nodes doesn't manipulate it
        self._hw_kill_listener = AlarmListener('hw-kill', callback_funct=self.on_hw_kill)
        # Provide service for alarm handler to set/clear the motherboard kill
        self._unkill_service = rospy.Service('/set_mobo_kill', SetBool, self.set_mobo_kill)
        # Sends hearbeat to board
        self._hearbeat_timer = rospy.Timer(rospy.Duration(0.5), self.send_heartbeat)
        # Create a subscribe for each thruster
        self._subs = [rospy.Subscriber(thruster, Float64, self.on_command, queue_size=10, callback_args=thruster)
                      for thruster in ThrustPacket.ID_MAPPING]

    def set_mobo_kill(self, req):
        '''
        Called on service calls to /set_mobo_kill, sending the approrpriate packet to the board
        to unassert or assert to motherboard-origin kill
        '''
        self.send_data(KillMessage.create_kill_message(command=True, hard=False, asserted=req.data).to_bytes(),
                       can_id=KILL_SEND_ID)
        return {'success': True}

    def send_heartbeat(self, timer):
        '''
        Send the special heartbeat packet when the timer triggers
        '''
        self.send_data(HeartbeatMessage.create().to_bytes(), can_id=KILL_SEND_ID)

    def on_hw_kill(self, alarm):
        '''
        Update the classe's hw-kill alarm to the latest update
        '''
        self._last_hw_kill = alarm

    def on_command(self, msg, thruster):
        '''
        When a thrust command message is received from the Subscriber, send the appropriate packet
        to the board
        '''
        packet = ThrustPacket.create_thrust_packet(ThrustPacket.ID_MAPPING[thruster], msg.data)
        self.send_data(packet.to_bytes(), can_id=THRUST_SEND_ID)

    def update_hw_kill(self):
        '''
        If needed, update the hw-kill alarm so it reflects the latest status from the board
        '''
        if self._last_soft_kill is None or self._last_hard_kill is None:
            return

        # Set serverity / problem message appropriately
        severity = 0
        message = ""
        if self._last_hard_kill is True:
            severity = 2
            message = "Hard kill"
        elif self._last_soft_kill is True:
            severity = 1
            message = "Soft kill"
        raised = severity != 0

        # If the current status differs from the alarm, update the alarm
        if self._last_hw_kill is None or self._last_hw_kill.raised != raised or \
           self._last_hw_kill.problem_description != message or self._last_hw_kill.severity != severity:
            if raised:
                self._kill_broadcaster.raise_alarm(severity=severity, problem_description=message)
            else:
                self._kill_broadcaster.clear_alarm(severity=severity, problem_description=message)

    def on_data(self, data):
        if KillMessage.IDENTIFIER == ord(data[0]):
            msg = KillMessage.from_bytes(data)
            if not msg.is_response:
                rospy.logwarn('Recieved kill message from board but was not response')
                return
            if msg.is_soft:
                self._last_soft_kill = msg.is_asserted
            elif msg.is_hard:
                self._last_hard_kill = msg.is_asserted
            self.update_hw_kill()
        elif GoMessage.IDENTIFIER == ord(data[0]):
            msg = GoMessage.from_bytes(data)
        else:
            rospy.logwarn('UNEXPECTED MESSAGE with identifier {}'.format(ord(data[0])))
            return
        rospy.loginfo(str(msg))
