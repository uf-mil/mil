#!/usr/bin/python
import rospy
from mil_usb_to_can import CANDeviceHandle
from thruster import make_thruster_dictionary
from packets import ThrustPacket, GoMessage, KillMessage, HeartbeatMessage, THRUST_SEND_ID, KILL_SEND_ID
from std_srvs.srv import SetBool
from ros_alarms import AlarmBroadcaster, AlarmListener
from sub8_msgs.msg import Thrust


class ThrusterAndKillBoard(CANDeviceHandle):
    '''
    Device handle for the thrust and kill board.
    '''

    def __init__(self, *args, **kwargs):
        super(ThrusterAndKillBoard, self).__init__(*args, **kwargs)
        # Initialize thruster mapping from params
        self.thrusters = make_thruster_dictionary(
            rospy.get_param('/thruster_layout/thrusters'))
        # Tracks last hw-kill alarm update
        self._last_hw_kill = None
        # Tracks last soft kill status received from board
        self._last_soft_kill = None
        # Tracks last hard kill status received from board
        self._last_hard_kill = None
        # Tracks last go status broadcasted
        self._last_go = None
        # Used to raise/clear hw-kill when board updates
        self._kill_broadcaster = AlarmBroadcaster('hw-kill')
        # Alarm broadaster for GO command
        self._go_alarm_broadcaster = AlarmBroadcaster('go')
        # Listens to hw-kill updates to ensure another nodes doesn't manipulate it
        self._hw_kill_listener = AlarmListener(
            'hw-kill', callback_funct=self.on_hw_kill)
        # Provide service for alarm handler to set/clear the motherboard kill
        self._unkill_service = rospy.Service(
            '/set_mobo_kill', SetBool, self.set_mobo_kill)
        # Sends hearbeat to board
        self._hearbeat_timer = rospy.Timer(
            rospy.Duration(0.4), self.send_heartbeat)
        # Create a subscribe for thruster commands
        self._sub = rospy.Subscriber(
            '/thrusters/thrust', Thrust, self.on_command, queue_size=10)

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
        self.send_data(HeartbeatMessage.create().to_bytes(),
                       can_id=KILL_SEND_ID)

    def on_hw_kill(self, alarm):
        '''
        Update the classe's hw-kill alarm to the latest update
        '''
        self._last_hw_kill = alarm

    def on_command(self, msg):
        '''
        When a thrust command message is received from the Subscriber, send the appropriate packet
        to the board
        '''
        for cmd in msg.thruster_commands:
            # If we don't have a mapping for this thruster, ignore it
            if cmd.name not in self.thrusters:
                rospy.logwarn(
                    'Command received for {}, but this is not a thruster.'.format(cmd.name))
                continue
            # Map commanded thrust (in newetons) to effort value (-1 to 1)
            effort = self.thrusters[cmd.name].effort_from_thrust(cmd.thrust)
            # Send packet to command specified thruster the specified force
            packet = ThrustPacket.create_thrust_packet(
                ThrustPacket.ID_MAPPING[cmd.name], effort)
            self.send_data(packet.to_bytes(), can_id=THRUST_SEND_ID)

    def update_hw_kill(self, reason):
        '''
        If needed, update the hw-kill alarm so it reflects the latest status from the board
        '''
        if self._last_soft_kill is None and self._last_hard_kill is None:
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
        message = message + ": " + reason

        # If the current status differs from the alarm, update the alarm
        if self._last_hw_kill is None or self._last_hw_kill.raised != raised or \
           self._last_hw_kill.problem_description != message or self._last_hw_kill.severity != severity:
            if raised:
                self._kill_broadcaster.raise_alarm(
                    severity=severity, problem_description=message)
            else:
                self._kill_broadcaster.clear_alarm(
                    severity=severity, problem_description=message)

    def on_data(self, data):
        '''
        Parse the two bytes and raise kills according to the specs:

        First Byte => Kill trigger statuses (Asserted = 1, Unasserted = 0):
            Bit 7: Hard kill status, changed by On/Off Hall effect switch. If this becomes 1, begin shutdown procedure
            Bit 6: Overall soft kill status, 1 if any of the following flag bits are 1. This becomes 0 if all kills are cleared and the thrusters are done re-initializing
            Bit 5: Hall effect soft kill flag
            Bit 4: Mobo command soft kill flag
            Bit 3: Heartbeat lost flag (times out after 1 s)
            Bit 2-0: Reserved
        Second Byte => Input statuses ("Pressed" = 1, "Unpressed" = 0) and thruster initializing status:
            Bit 7: On (1) / Off (0) Hall effect switch status. If this becomes 0, the hard kill in the previous byte becomes 1.
            Bit 6: Soft kill Hall effect switch status. If this is "pressed" = 1, bit 5 of previous byte is unkilled = 0. "Removing" the magnet will "unpress" the switch
            Bit 5: Go Hall effect switch status. "Pressed" = 1, removing the magnet will "unpress" the switch
            Bit 4: Thruster initializing status: This becomes 1 when the board is unkilling, and starts powering thrusters. After the "grace period" it becomes 0 and the overall soft kill status becomes 0. This flag also becomes 0 if killed in the middle of initializing thrusters.
            Bit 3-0: Reserved
        '''
        #print(ord(data[0]), ord(data[1]))
        # Hard Kill - bit 7
        if (ord(data[0]) & 0x8):
            self._last_hard_kill = True
        else:
            self._last_hard_kill = False
        # Soft Kill - bit 6
        if (ord(data[0]) & 0x40):
            self._last_soft_kill = True
        else:
            self._last_soft_kill = False

        reason = ""
        # hall effect - bit 5
        if (ord(data[0]) & 0x20):
            reason = 'hall effect'
        # mobo soft kill - bit 4
        if (ord(data[0]) & 0x10):
            reason = 'mobo soft kill'
        # heartbeat loss - bit 3
        if (ord(data[0]) & 0x08):
            reason = 'Heart beat lost'
        self.update_hw_kill(reason)

        # Switch - bit 5
        if (ord(data[1]) & 0x20):
            asserted = True
        else:
            asserted = False
        if self._last_go is None or asserted != self._last_go:
            if asserted:
                self._go_alarm_broadcaster.raise_alarm(
                    problem_description="Go plug pulled!")
            else:
                self._go_alarm_broadcaster.clear_alarm(
                    problem_description="Go plug returned")
            self._last_go = asserted
