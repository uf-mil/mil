#!/usr/bin/python
import rospy
from mil_usb_to_can import CANDeviceHandle
from packets import ThrustPacket, GoMessage, KillMessage
from std_msgs.msg import Float64
from ros_alarms import AlarmBroadcaster, AlarmListener


class ThrusterAndKillBoard(CANDeviceHandle):
    '''
    TODO
    '''
    def __init__(self, *args, **kwargs):
        super(ThrusterAndKillBoard, self).__init__(*args, **kwargs)
        self.correct_response = 37
        self.response_received = None
        self._last_kill = None
        self.kill_listener = AlarmListener('kill', callback_funct=self.on_soft_kill)
        self._hard_kill_broadcaster = AlarmBroadcaster('hw-hard-kill')
        self._soft_kill_broadcaster = AlarmBroadcaster('kill')

        self._subs = [rospy.Subscriber(thruster, Float64, self.on_command, queue_size=10, callback_args=thruster)
                      for thruster in ThrustPacket.ID_MAPPING]

    def on_soft_kill(self, alarm):
        if self._last_kill is not alarm.raised:
            self.send_data(KillMessage.create_kill_message(command=True, hard=False, asserted=alarm.raised).to_bytes())
        self._last_kill = alarm.raised

    def on_command(self, msg, thruster):
        rospy.loginfo('Commanding {} {}'.format(thruster, msg.data))
        packet = ThrustPacket.create_thrust_packet(ThrustPacket.ID_MAPPING[thruster], msg.data)
        self.send_data(packet.to_bytes(), can_id=0x21)

    def on_data(self, data):
        if KillMessage.IDENTIFIER == ord(data[0]):
            msg = KillMessage.from_bytes(data)
            if not msg.is_response:
                rospy.logwarn('Recieved kill message from board but was not response')
                return
            if msg.is_hard:
                if msg.is_asserted:
                    rospy.logwarn('HARD KILL RAISED! SYSTEM WILL SHUT DOWN!')
                    self._hard_kill_broadcaster.raise_alarm(problem_description='Hard kill raised by kill board')
                else:
                    self._hard_kill_broadcaster.clear_alarm(problem_description='Hard kill cleared by kill board')
            elif msg.is_soft:
                if msg.is_asserted and self._last_kill is False:
                    self._soft_kill_broadcaster.raise_alarm(problem_description='Kill raised by board')

        elif GoMessage.IDENTIFIER == ord(data[0]):
            msg = GoMessage.from_bytes(data)
        else:
            rospy.logwarn('UNEXPECTED MESSAGE with identifier {}'.format(data[0]))
        rospy.loginfo(str(msg))
