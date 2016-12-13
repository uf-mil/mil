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
        
        self.need_kill = False
        self.killed = False
        # Initial check of kill status
        self.get_status() 

        oelf.current_wrencher = ''
        _set_wrencher = lambda msg: setattr(self, 'current_wrencher', msg.data)
        rospy.Subscriber("/wrench/current", String, _set_wrencher)

        al = AlarmListener("kill", self.alarm_kill_cb)
        
        # Dict of op-codes to functions that need to be run with each code for aysnc responses.
        self.update_cbs = {'\x10': self.set_kill, '\x11': self.set_unkill, 
                           '\x12': lambda: True, '\x13': lambda: True, '\x14': lambda: True, '\x15': lambda: True,
                           '\x16': lambda: True, '\x17': lambda: True, '\x18': lambda: True, '\x19': lambda: True,
                           '\x1A': lambda: True, '\x1B': lambda: True,
                           '\x1C': lambda: True, '\x1D': lambda: True,
                           '\x1E': self.disconnect.clear_alarm, '\x1F': self.disconnect.raise_alarm,}
        
        while not rospy.is_shutdown():
            rospy.sleep(.5)
            while self.ser.inWaiting() > 0:
                self.check_buffer()
            self.get_status()
            self.control_check()
            if not self.network_kill():
                self.ping()

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
        if resp in self.update_cbs:
            fprint(self.to_hex(resp), title="CHECKBUFFER")
            self.update_cbs[resp]()

    @thread_lock(lock)
    def request(self, write_str, recv_str=None):
        """
        Deals with requesting data and checking if the response matches some `recv_str`.
        Returns True or False depending on the response.
        With no `recv_str` passed in the raw result will be returned.
        """
        fprint("Writing {}...".format(self.to_hex(write_str)), title="REQUEST")
        self.ser.write(write_str)
    
        fprint("Reading response...", title="REQUEST")
        resp = self.ser.read(1)
        
        rospy.sleep(.05)
        if recv_str is None:
            fprint("Response received: {}".format(self.to_hex(resp)), msg_color='blue')
            return resp
        
        if resp in recv_str:
            # It matched!
            fprint("Response matched!", title="REQUEST", msg_color='green')
            return True

        self.ser.flushOutput()
        # Result didn't match
        fprint("Response didn't match. Expected: {}, got: {}.".format(self.to_hex(recv_str), self.to_hex(resp)), msg_color='red')
        return False

    def alarm_kill_cb(self, alarm):
        # Ignore the alarm if it came from us
        if alarm.node_name == rospy.get_name() and not alarm.clear:
            return
        
        if not alarm.clear:
            self.request('\x45')
        else:
            self.request('\x46')
    
    def control_check(self, *args):
        # Update status light with current control
        
        if self.current_wrencher == 'autonomous':
            self.request('\x42', '\x52')
        elif self.current_wrencher in ['keyboard', 'rc']:
            self.request('\x41', '\x51')
        else:
            self.request('\x40', '\x50')

    def get_status(self):
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
        fprint("Pinging...")
        
        if self.request('\x20', '\x30'):
            fprint("Ping response!", msg_color='green')
        else:
            fprint("No ping response found", msg_color='red')



if __name__ == '__main__':
    rospy.init_node("kill_interface")
    k = KillInterface() 
    rospy.spin()
