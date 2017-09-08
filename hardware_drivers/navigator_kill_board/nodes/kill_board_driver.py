#!/usr/bin/env python
import rospy
import threading
import serial
from std_msgs.msg import Header, String
from mil_tools import thread_lock
from ros_alarms import AlarmBroadcaster, AlarmListener
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from navigator_kill_board import constants

lock = threading.Lock()


class KillInterface(object):
    """
    Driver to interface with NaviGator's kill handeling board, which disconnects power to actuators
    if any of 4 emergency buttons is pressed, a software kill command is sent, or the network hearbeat
    stops. This driver enables the software kill option via ros_alarms and outputs diagnostics
    data about the board to ROS.
    """

    ALARM = 'hw-kill'  # Alarm to raise when hardware kill is detected
    YELLOW_WRENCHES = ['rc', 'keyboard']  # List of wrenches which will activate yellow diagnostic light
    GREEN_WRENCHES = ['autonomous']  # List of wrenches which will activate green diagnostic light

    def __init__(self):
        self.port = rospy.get_param('~port', "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A104OWRY-if00-port0")
        if rospy.get_param('/is_simulation', False):  # If in Gazebo, run fake serial class following board's protocol
            from navigator_kill_board import SimulatedKillBoard
            self.ser = SimulatedKillBoard()
        else:
            baud = rospy.get_param('~baud', 9600)
            self.ser = serial.Serial(port=self.port, baudrate=baud)
        self.ser.flush()

        self.board_status = {}
        for kill in constants['KILLS']:
            self.board_status[kill] = False
        self.network_msg = None
        self.wrench = ''
        self._hw_killed = False

        self.hw_kill_broadcaster = AlarmBroadcaster('hw-kill')
        self.diagnostics_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=3)

        AlarmListener('hw-kill', self.hw_kill_alarm_cb)
        AlarmListener('kill', self.kill_alarm_cb)
        rospy.Subscriber("/wrench/current", String, self.wrench_cb)
        rospy.Subscriber("/network", Header, self.network_cb)  # Passes along network hearbeat to kill board

    def run(self):
        '''
        Main loop for driver, at a fixed rate updates alarms and diagnostics output with new
        kill board output.
        '''
        self.receive_async()
        self.request_update()
        self.update_ros()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.receive_async()
            self.update_ros()
            rate.sleep()

    def update_ros(self):
        self.update_hw_kill()
        self.publish_diagnostics()

    @thread_lock(lock)
    def receive_async(self):
        '''
        Recieve update bytes sent from the board without requests being sent, updating internal
        state, raising alarms, etc in response to board updates.
        '''
        while self.ser.in_waiting > 0 and not rospy.is_shutdown():
            msg = self.ser.read(1)
            for kill in self.board_status:
                if msg == constants[kill]['FALSE']:
                    self.board_status[kill] = False
                if msg == constants[kill]['TRUE']:
                    self.board_status[kill] = True

    def request_update(self):
        '''
        Manually request status of all kills, used on program startup to
        get up to date with current state
        '''
        for key in self.board_status:
            res = self.request(constants[key]['REQUEST'])
            if res == constants['RESPONSE_FALSE']:
                self.board_status[key] = False
            elif res == constants['RESPONSE_TRUE']:
                self.board_status[key] = True
            else:
                print 'got unexpected response on kill request'

    def wrench_cb(self, msg):
        '''
        Updates wrench (autnomous vs teleop) diagnostic light if nessesary
        on wrench changes
        '''
        wrench = msg.data
        if wrench != self.wrench:
            if wrench in self.YELLOW_WRENCHES:
                self.request(constants['LIGHTS']['YELLOW_REQUEST'])
            elif wrench in self.GREEN_WRENCHES:
                self.request(constants['LIGHTS']['GREEN_WRENCHES'])
            else:
                self.request(constants['LIGHTS']['OFF_REQUEST'])
            self.wrench = wrench

    def network_cb(self, msg):
        '''
        Pings kill board on every network hearbeat message. Pretends to be the rf-based hearbeat because
        real one does not work :(
        '''
        self.request(constants['PING']['REQUEST'])

    def publish_diagnostics(self):
        '''
        Publishes current kill board state in ROS diagnostics package, making it easy to use in GUIs and other ROS tools
        '''
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = 'kill_board'
        status.hardware_id = self.port
        for key, value in self.board_status.items():
            status.values.append(KeyValue(key, str(value)))
        msg.status.append(status)
        self.diagnostics_pub.publish(msg)

    def update_hw_kill(self):
        '''
        Raise/Clear hw-kill ROS Alarm is nessesary (any kills on board are engaged)
        '''
        killed =  any([self.board_status[key] for key in self.board_status])
        if killed and not self._hw_killed:
            self._hw_killed = True
            self.hw_kill_broadcaster.raise_alarm(parameters=self.board_status)
        if not killed and self._hw_killed:
            self._hw_killed = False
            self.hw_kill_broadcaster.clear_alarm()

    @thread_lock(lock)
    def request(self, write_str, expected_response=None):
        """
        Deals with requesting data and checking if the response matches some `recv_str`.
        Returns True or False depending on the response.
        With no `recv_str` passed in the raw result will be returned.
        """
        self.ser.write(write_str)
        resp = self.ser.read(1)

        #self.ser.reset_input_buffer()
        #self.ser.reset_output_buffer()

        if expected_response is None:
            return resp

        if resp == expected_response:
            return True
        else:
            rospy.logerr("Response didn't match. Expected: {}, got: {}.".format(hex(ord(write_str)), hex(ord(resp))))
            return False

    def kill_alarm_cb(self, alarm):
        '''
        Informs kill board about software kills through ROS Alarms
        '''
        if alarm.raised:
            self.request(constants['COMPUTER']['KILL']['REQUEST'])
        else:
            self.request(constants['COMPUTER']['CLEAR']['REQUEST'])

    def hw_kill_alarm_cb(self, alarm):
        self._hw_killed = alarm.raised


if __name__ == '__main__':
    rospy.init_node("kill_board_driver")
    driver = KillInterface()
    driver.run()
