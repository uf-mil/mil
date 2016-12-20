#!/usr/bin/env python
import rospy

import threading
import serial

from std_msgs.msg import String
from std_msgs.msg import Header

from navigator_tools import thread_lock
from navigator_tools import fprint as _fprint
from navigator_alarm import AlarmBroadcaster, AlarmListener
from navigator_msgs.msg import KillStatus

fprint = lambda *args, **kwargs: _fprint(time='', *args, **kwargs)
lock = threading.Lock()

class KillInterface(object):
    """
    This handles the comms node between ROS and kill/status embedded board.
    There are two things running here:
        1. From ROS: Check current operation mode of the boat and tell that to the light
        2. From BASE: Check the current kill status from the other sources
    """

    def __init__(self, port="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A104OWRY-if00-port0", baud=9600):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.25)
        self.ser.flush()
        
        self.timeout = rospy.Duration(1)
        self.network_msg = None
        update_network = lambda msg: setattr(self, "network_msg", msg)
        self.network_listener = rospy.Subscriber("/keep_alive", Header, update_network)
        
        self.killstatus_pub = rospy.Publisher("/killstatus", KillStatus, queue_size=1)

        ab = AlarmBroadcaster()
        self.kill_alarm = ab.add_alarm("hw_kill", problem_description="Hardware kill from a kill switch.")
        self.disconnect = ab.add_alarm("kill_system_disconnect")
        
        self.killed = False
        
        self.kill_status = {'overall': False, 'PF': False, 'PA': False, 'SF': False, 'SA': False, 'computer': False, 'remote': False}
        # Dict of op-codes to functions that need to be run with each code for aysnc responses
        self.update_cbs = {'\x10': lambda: self.update('overall', True), '\x11': lambda: self.update('overall', False), 
                           '\x12': lambda: self.update('PF', True), '\x13': lambda: self.update('PF', False), 
                           '\x14': lambda: self.update('PA', True), '\x15': lambda: self.update('PA', False),
                           '\x16': lambda: self.update('SF', True), '\x17': lambda: self.update('SF', False), 
                           '\x18': lambda: self.update('SA', True), '\x19': lambda: self.update('SA', False),
                           '\x1A': lambda: self.update('remote', True), '\x1B': lambda: self.update('remote', False),
                           '\x1C': lambda: self.update('computer', True), '\x1D': lambda: self.update('computer', False),
                           '\x1E': self.disconnect.clear_alarm, '\x1F': self.disconnect.raise_alarm}

        # Initial check of kill status
        self.get_status() 

        self.current_wrencher = ''
        _set_wrencher = lambda msg: setattr(self, 'current_wrencher', msg.data)
        rospy.Subscriber("/wrench/current", String, _set_wrencher)

        al = AlarmListener("kill", self.alarm_kill_cb)
        
        rospy.Timer(rospy.Duration(.5), self.get_status)

        while not rospy.is_shutdown():
            self.control_check()

            while self.ser.inWaiting() > 0 and not rospy.is_shutdown():
                self.check_buffer()

            if not self.network_kill():
                self.ping()
            else:
                rospy.logerr("KILLBOARD: Network Kill!") 

    def update(self, src, status):
        rospy.loginfo("Updating... from {}: {}".format(src, status))
        self.kill_status[src] = status

    def network_kill(self):
        if self.network_msg is None:
           return False

        return ((rospy.Time.now() - self.network_msg.stamp) > self.timeout)

    def to_hex(self, arg):
        ret = '\x99'
        try:
            ret = hex(ord(arg))
        except Exception as e:
            rospy.logerr(e)
            self.ser.flushInput()
            self.ser.flushOutput()

        return ret

    def set_kill(self):
        self.killed = True
        self.kill_alarm.raise_alarm()

    def set_unkill(self):
        self.killed = False
        self.kill_alarm.clear_alarm()

    @thread_lock(lock)
    def check_buffer(self):
        # The board appears to not be return async data
        resp = self.ser.read(1)
        print resp.encode('hex')
        if resp in self.update_cbs:
            rospy.loginfo("Check Buffer response: {}".format(resp.encode('hex')))
            self.update_cbs[resp]()

    @thread_lock(lock)
    def request(self, write_str, recv_str=None):
        """
        Deals with requesting data and checking if the response matches some `recv_str`.
        Returns True or False depending on the response.
        With no `recv_str` passed in the raw result will be returned.
        """
        rospy.loginfo("Writing {}".format(write_str.encode('hex')))
        self.ser.write(write_str)
        rospy.sleep(.1)

        return True 

        resp = self.ser.read(1) 
        
        rospy.loginfo("Sent: {}, Rec: {}".format(write_str.encode('hex'), resp.encode('hex')))

        rospy.sleep(.05)
        if recv_str is None:
            #fprint("Response received: {}".format(self.to_hex(resp)), msg_color='blue')
            return resp
        
        if resp in recv_str:
            # It matched!
            fprint("Response matched!", title="REQUEST", msg_color='green')
            return True

        self.ser.flushOutput()
        # Result didn't match
        rospy.logerr("Response didn't match. Expected: {}, got: {}.".format(recv_str.encode('hex'), resp.encode('hex')))
        return False

    def alarm_kill_cb(self, alarm):
        # Ignore the alarm if it came from us
        if alarm.node_name == rospy.get_name() and not alarm.clear:
            return
        
        if not alarm.clear:
            rospy.loginfo("Computer kill raise received")
            self.request('\x45')
        else:
            rospy.loginfo("Computer kill clear received")
            self.request('\x46')

    def control_check(self, *args):
        # Update status light with current control
        
        if self.current_wrencher == 'autonomous':
            self.request('\x42', '\x52')
        elif self.current_wrencher in ['keyboard', 'rc']:
            self.request('\x41', '\x51')
        else:
            self.request('\x40', '\x50')

    def get_status(self, *args):
        killstatus = KillStatus()
        killstatus.overall = self.kill_status['overall']
        killstatus.pf = self.kill_status['PF']
        killstatus.pa = self.kill_status['PA'] 
        killstatus.sf = self.kill_status['SF']
        killstatus.sa = self.kill_status['SA']
        killstatus.remote = self.kill_status['remote']
        killstatus.computer = self.kill_status['computer']
        # killstatus.remote_conn = ord(remote_conn) == 1
        self.killstatus_pub.publish(killstatus)

        # If any of the kill options (except the computer) are true, raise the alarm.
        if any([killstatus.pf, killstatus.pa, killstatus.sf, killstatus.sa, killstatus.remote]):
            self.set_kill()
        else:
            self.set_unkill()

    def _get_status(self):
        """
        Request an updates all current status indicators
        """
        # Overall kill status
        overall = self.request('\x21')
        pf = self.request('\x22')
        pa = self.request('\x23')
        sf = self.request('\x24')
        sa = self.request('\x25')
        remote = self.request('\x26')
        computer = self.request('\x27')
        # remote_conn = self.request('\x28')
        
        try:
            killstatus = KillStatus()
            killstatus.overall = ord(overall) == 1
            killstatus.pf = ord(pf) == 1
            killstatus.pa = ord(pa) == 1
            killstatus.sf = ord(sf) == 1
            killstatus.sa = ord(sa) == 1
            killstatus.remote = ord(remote) == 1
            killstatus.computer = ord(computer) == 1
            # killstatus.remote_conn = ord(remote_conn) == 1
            self.killstatus_pub.publish(killstatus)

            # self.need_kill = ord(remote_conn) == 0 
        except Exception as e:
            rospy.logerr(e)
            self.ser.flushInput()
            self.ser.flushOutput()

        # If any of the kill options (except the computer) are true, raise the alarm.
        if 5 >= sum(map(ord, [pf, pa, sf, sa, remote])) >= 1:
            self.set_kill()
        else:
            self.set_unkill()

    def ping(self):
        #rospy.loginfo("Pinging")
        self.request('\x20') 
        # if self.request('\x20', '\x30'):
        #     #rospy.loginfo("Ponged")
        # else:
        #     rospy.logerr("Incorrect ping response")


if __name__ == '__main__':
    rospy.init_node("kill_interface")
    k = KillInterface() 
    rospy.spin()
