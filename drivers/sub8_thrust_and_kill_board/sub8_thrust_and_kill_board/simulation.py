#!/usr/bin/python
from mil_usb_to_can import SimulatedCANDevice
from packets import ThrustPacket, GoMessage, KillMessage, HeartbeatMessage, THRUST_SEND_ID, KILL_SEND_ID
import rospy
from std_srvs.srv import SetBool


class ThrusterAndKillBoardSimulation(SimulatedCANDevice):
    '''
    Serial simulator for the thruster and kill board,
    providing services to simulate physical plug connections/disconnections
    '''
    HEARTBEAT_TIMEOUT_SECONDS = rospy.Duration(1.0)

    def __init__(self, *args, **kwargs):
        self.hard_kill_plug_pulled = False
        self.hard_kill_mobo = False
        self.soft_kill_plug_pulled = False
        self.soft_kill_mobo = False
        self.go_button = False
        self._last_heartbeat = None
        super(ThrusterAndKillBoardSimulation, self).__init__(*args, **kwargs)
        self._update_timer = rospy.Timer(rospy.Duration(1), self.send_updates)
        self._soft_kill = rospy.Service('/simulate_soft_kill', SetBool, self.set_soft_kill)
        self._hard_kill = rospy.Service('/simulate_hard_kill', SetBool, self.set_hard_kill)
        self._go_srv = rospy.Service('/go', SetBool, self._on_go_srv)

    def _on_go_srv(self, req):
        self.go_button = req.data
        return {'success': True}

    def set_soft_kill(self, req):
        self.soft_kill_plug_pulled = req.data
        return {'success': True}

    def set_hard_kill(self, req):
        self.hard_kill_plug_pulled = req.data
        return {'success': True}

    @property
    def hard_killed(self):
        return self.hard_kill_mobo or self.hard_kill_plug_pulled

    @property
    def soft_killed(self):
        return self.soft_kill_plug_pulled or self.soft_kill_mobo or self._last_heartbeat is None or \
            (rospy.Time.now() - self._last_heartbeat) > self.HEARTBEAT_TIMEOUT_SECONDS

    def send_updates(self, *args):
        hard_msg = KillMessage.create_kill_message(command=False, hard=True, asserted=self.hard_killed)
        soft_msg = KillMessage.create_kill_message(command=False, hard=False, asserted=self.soft_killed)
        go_msg = GoMessage.create_go_message(asserted=self.go_button)
        #self.send_data(hard_msg.to_bytes())
        #self.send_data(soft_msg.to_bytes())
        #self.send_data(go_msg.to_bytes())

    def on_data(self, data, can_id):
        if can_id != THRUST_SEND_ID and can_id != KILL_SEND_ID:
            return
        if KillMessage.IDENTIFIER == ord(data[0]):
            packet = KillMessage.from_bytes(data)
            assert packet.is_command
            if packet.is_hard:
                self.hard_kill_mobo = packet.is_asserted
            elif packet.is_soft:
                self.soft_kill_mobo = packet.is_asserted
            else:
                assert False
            self.send_updates()
        elif ThrustPacket.IDENTIFIER == ord(data[0]):
            packet = ThrustPacket.from_bytes(data)
        elif HeartbeatMessage.IDENTIFIER == ord(data[0]):
            packet = HeartbeatMessage.from_bytes(data)
            self._last_heartbeat = rospy.Time.now()
        else:
            assert False
