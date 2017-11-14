#!/usr/bin/env python
import rospy
import copy
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
    data about the board to ROS. The driver can handle the board's asyncronous updates of kill statuses
    and will also periodicly request updates in case the async is not working (it often doesn't).
    """

    ALARM = 'hw-kill'  # Alarm to raise when hardware kill is detected
    YELLOW_WRENCHES = ['rc', '/wrench/rc', 'keyboard', '/wrench/keyboard']  # Wrenches which activate YELLOW LED
    GREEN_WRENCHES = ['autonomous', '/wrench/autonomous']  # Wrenches which activate GREEN LED

    def __init__(self):
        self.port = rospy.get_param('~port', "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A104OWRY-if00-port0")
        self.connected = False
        self.diagnostics_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=3)
        while not self.connected and not rospy.is_shutdown():
            try:
                self.connect()
                self.connected = True
            except serial.SerialException as e:
                rospy.logerr('Cannot connect to kill board. {}'.format(e))
                self.publish_diagnostics(e)
                rospy.sleep(1)
        rospy.loginfo('Board connected!')
        self.board_status = {}
        for kill in constants['KILLS']:
            self.board_status[kill] = False
        self.kills = self.board_status.keys()
        self.expected_responses = []
        self.network_msg = None
        self.wrench = ''
        self._hw_killed = False
        self._last_hw_kill_paramaters = self.board_status
        self.last_request = None
        self.request_index = -1

        self.hw_kill_broadcaster = AlarmBroadcaster('hw-kill')

        AlarmListener('hw-kill', self.hw_kill_alarm_cb)
        AlarmListener('kill', self.kill_alarm_cb)
        rospy.Subscriber("/wrench/selected", String, self.wrench_cb)
        rospy.Subscriber("/network", Header, self.network_cb)  # Passes along network hearbeat to kill board

    def connect(self):
        if rospy.get_param('/is_simulation', False):  # If in Gazebo, run fake serial class following board's protocol
            from navigator_kill_board import SimulatedKillBoard
            self.ser = SimulatedKillBoard()
        else:
            baud = rospy.get_param('~baud', 9600)
            self.ser = serial.Serial(port=self.port, baudrate=baud)

    def run(self):
        '''
        Main loop for driver, at a fixed rate updates alarms and diagnostics output with new
        kill board output.
        '''
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.last_request is None:
                self.request_next()
            self.receive()
            self.update_ros()
            rate.sleep()

    def update_ros(self):
        self.update_hw_kill()
        self.publish_diagnostics()

    def handle_byte(self, msg):
        '''
        React to a byte recieved from the board. This could by an async update of a kill status or
        a known response to a recent request
        '''
        # If a response has been recieved to a requested status (button, remove, etc), update internal state
        if self.last_request is not None:
            if msg == constants['RESPONSE_FALSE']:
                self.board_status[self.last_request] = False
                self.last_request = None
                return
            if msg == constants['RESPONSE_TRUE']:
                self.board_status[self.last_request] = True
                self.last_request = None
                return
        # If an async update was recieved, update internal state
        for kill in self.board_status:
            if msg == constants[kill]['FALSE']:
                self.board_status[kill] = False
                return
            if msg == constants[kill]['TRUE']:
                self.board_status[kill] = True
                return
        # If a response to another request, like ping or computer kill/clear is recieved
        for index, byte in enumerate(self.expected_responses):
            if msg == byte:
                del self.expected_responses[index]
                return
        # Log a warning if an unexpected byte was recieved
        rospy.logwarn('Recieved an unexpected byte {}, remaining expected_responses={}'.format(
            hex(ord(msg)), len(self.expected_responses)))

    @thread_lock(lock)
    def receive(self):
        '''
        Recieve update bytes sent from the board without requests being sent, updating internal
        state, raising alarms, etc in response to board updates. Clears the in line buffer.
        '''
        while self.ser.in_waiting > 0 and not rospy.is_shutdown():
            msg = self.ser.read(1)
            self.handle_byte(msg)

    def request_next(self):
        '''
        Manually request status of the next kill, looping back to the first
        once all have been responsded to.
        '''
        self.request_index += 1
        if self.request_index == len(self.kills):
            self.request_index = 0
        self.last_request = self.kills[self.request_index]
        self.request(constants[self.last_request]['REQUEST'])

    def wrench_cb(self, msg):
        '''
        Updates wrench (autnomous vs teleop) diagnostic light if nessesary
        on wrench changes
        '''
        wrench = msg.data
        if wrench != self.wrench:
            if wrench in self.YELLOW_WRENCHES:
                self.request(constants['LIGHTS']['YELLOW_REQUEST'], constants['LIGHTS']['YELLOW_RESPONSE'])
            elif wrench in self.GREEN_WRENCHES:
                self.request(constants['LIGHTS']['GREEN_REQUEST'], constants['LIGHTS']['GREEN_RESPONSE'])
            else:
                self.request(constants['LIGHTS']['OFF_REQUEST'], constants['LIGHTS']['OFF_RESPONSE'])
            self.wrench = wrench

    def network_cb(self, msg):
        '''
        Pings kill board on every network hearbeat message. Pretends to be the rf-based hearbeat because
        real one does not work :(
        '''
        self.request(constants['PING']['REQUEST'], constants['PING']['RESPONSE'])

    def publish_diagnostics(self, err=None):
        '''
        Publishes current kill board state in ROS diagnostics package, making it easy to use in GUIs and other ROS tools
        '''
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = 'kill_board'
        status.hardware_id = self.port
        if not self.connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Cannot connect to kill board. Retrying every second.'
            status.values.append(KeyValue("Exception", str(err)))
        else:
            status.level = DiagnosticStatus.OK
            for key, value in self.board_status.items():
                status.values.append(KeyValue(key, str(value)))
        msg.status.append(status)
        self.diagnostics_pub.publish(msg)

    def update_hw_kill(self):
        '''
        Raise/Clear hw-kill ROS Alarm is nessesary (any kills on board are engaged)
        '''
        killed = any([self.board_status[key] for key in self.board_status])
        if (killed and not self._hw_killed) or (killed and self.board_status != self._last_hw_kill_paramaters):
            self._hw_killed = True
            self.hw_kill_broadcaster.raise_alarm(parameters=self.board_status)
            self._last_hw_kill_paramaters = copy.copy(self.board_status)
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
        if expected_response is not None:
            for byte in expected_response:
                self.expected_responses.append(byte)

    def kill_alarm_cb(self, alarm):
        '''
        Informs kill board about software kills through ROS Alarms
        '''
        if alarm.raised:
            self.request(constants['COMPUTER']['KILL']['REQUEST'], constants['COMPUTER']['KILL']['RESPONSE'])
        else:
            self.request(constants['COMPUTER']['CLEAR']['REQUEST'], constants['COMPUTER']['CLEAR']['RESPONSE'])

    def hw_kill_alarm_cb(self, alarm):
        self._hw_killed = alarm.raised


if __name__ == '__main__':
    rospy.init_node("kill_board_driver")
    driver = KillInterface()
    driver.run()
