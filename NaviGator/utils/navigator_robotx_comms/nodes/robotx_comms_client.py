#!/usr/bin/env python

"""
RobotX Communications: A node that handles message communications
with the Technical Director server for the RobotX Communication Protocol
"""

import datetime
import socket
import threading

import rospy
from ros_alarms import AlarmListener
from geometry_msgs.msg import PointStamped
from mil_tools import thread_lock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from navigator_msgs.msg import ScanTheCode
from navigator_msgs.srv import MessageExtranceExitGate, MessageIdentifySymbolsDock, MessageDetectDeliver

from navigator_robotx_comms.navigator_robotx_comms import RobotXDetectDeliverMessage, RobotXHeartbeatMessage, \
    RobotXEntranceExitGateMessage, RobotXScanCodeMessage, RobotXIdentifySymbolsDockMessage
lock = threading.Lock()

rospy.init_node("robotx_comms_client")


class RobotXStartServices:
    """
    Initializes services and subscribes to necessary publishers
    """

    def __init__(self):
        # define all variables for subscribers
        self.gps_array = None
        self.odom = None
        self.auv_status = 1  # we don't have an AUV, so this will always be 1
        self.system_mode = None
        self.wrench = None
        self.kill = False
        # define delimiter for messages
        self.delim = ','
        # define parameters
        self.team_id = rospy.get_param("~team_id")
        self.td_ip = rospy.get_param("~td_ip")
        self.td_port = rospy.get_param("~td_port")
        self.use_test_data = rospy.get_param("~use_test_data")
        # time last called
        self.time_last_entrance_exit = None
        self.time_last_scan_code = None
        self.time_last_identify_symbols = None
        self.time_last_detect_deliver = None
        # initialize connection to server
        self.robotx_client = RobotXClient(self.td_ip, self.td_port)
        self.robotx_client.connect()

        # setup all message types
        self.robotx_heartbeat_message = RobotXHeartbeatMessage()
        self.robotx_entrance_exit_gate_message = RobotXEntranceExitGateMessage()
        self.robotx_scan_code_message = RobotXScanCodeMessage()
        self.robotx_identify_symbols_dock_message = RobotXIdentifySymbolsDockMessage()
        self.robotx_detect_deliver_message = RobotXDetectDeliverMessage()

        # setup all subscribers
        rospy.Subscriber("lla", PointStamped, self.gps_coord_callback)
        rospy.Subscriber("odom", Odometry, self.gps_odom_callback)
        rospy.Subscriber("/wrench/selected", String, self.wrench_callback)
        rospy.Subscriber("/scan_the_code", ScanTheCode, self.scan_the_code_callback)

        # track kill state for inferring system mode
        self.kill_listener = AlarmListener('kill', self.kill_callback)
        self.kill_listener.wait_for_server()

        # setup all services
        self.service_entrance_exit_gate_message = rospy.Service("entrance_exit_gate_message",
                                                                MessageExtranceExitGate,
                                                                self.handle_entrance_exit_gate_message)
        self.service_identify_symbols_dock_message = rospy.Service("identify_symbols_dock_message",
                                                                   MessageIdentifySymbolsDock,
                                                                   self.handle_identify_symbols_dock_message)
        self.service_detect_deliver_message = rospy.Service("detect_deliver_message",
                                                            MessageDetectDeliver,
                                                            self.handle_detect_deliver_message)

        # start sending heartbeat every second
        rospy.Timer(rospy.Duration(1), self.handle_heartbeat_message)

    def update_system_mode(self):
        if self.kill is True:
            self.system_mode = 3
        elif self.wrench == "autonomous" or self.wrench == "/wrench/autonomous":
            self.system_mode = 2
        else:
            self.system_mode = 1

    def wrench_callback(self, wrench):
        self.wrench = wrench.data

    def kill_callback(self, alarm):
        self.kill = alarm.raised

    def gps_coord_callback(self, lla):
        self.gps_array = lla

    def gps_odom_callback(self, odom):
        self.odom = odom

    def auv_status_callback(self, auv_status):
        self.auv_status = auv_status

    def system_mode_callback(self, system_mode):
        self.system_mode = system_mode

    def scan_the_code_callback(self, scan_the_code):
        self.handle_scan_code_message(scan_the_code.color_pattern)

    def handle_heartbeat_message(self, data):
        self.update_system_mode()
        hst_date_time = self.get_hst_date_time()

        message = self.robotx_heartbeat_message.to_string(self.delim, self.team_id, hst_date_time,
                                                          self.gps_array, self.odom, self.auv_status,
                                                          self.system_mode, self.use_test_data)
        self.robotx_client.send_message(message)

    def handle_entrance_exit_gate_message(self, data):
        if self.time_last_entrance_exit is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_entrance_exit
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_entrance_exit = rospy.get_time()
        hst_date_time = self.get_hst_date_time()
        message = self.robotx_entrance_exit_gate_message.to_string(self.delim, self.team_id, hst_date_time,
                                                                   data, self.use_test_data)
        self.robotx_client.send_message(message.message)
        return message

    def handle_scan_code_message(self, color_pattern):
        if self.time_last_scan_code is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_scan_code
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_scan_code = rospy.get_time()
        hst_date_time = self.get_hst_date_time()
        message = self.robotx_scan_code_message.to_string(self.delim, self.team_id, hst_date_time,
                                                          color_pattern, self.use_test_data)
        self.robotx_client.send_message(message)

    def handle_identify_symbols_dock_message(self, data):
        if self.time_last_identify_symbols is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_identify_symbols
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_identify_symbols = rospy.get_time()
        hst_date_time = self.get_hst_date_time()
        message = self.robotx_identify_symbols_dock_message.to_string(self.delim, self.team_id, hst_date_time,
                                                                      data, self.use_test_data)
        self.robotx_client.send_message(message.message)
        return message

    def handle_detect_deliver_message(self, data):
        if self.time_last_detect_deliver is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_detect_deliver
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_detect_deliver = rospy.get_time()
        hst_date_time = self.get_hst_date_time()
        message = self.robotx_detect_deliver_message.to_string(self.delim, self.team_id, hst_date_time,
                                                               data, self.use_test_data)
        self.robotx_client.send_message(message.message)
        return message

    def get_hst_date_time(self):
        # HST is 10 hours behind UTC
        hst_time = datetime.datetime.utcnow() - datetime.timedelta(hours=10, minutes=0)
        date_string = hst_time.strftime("%d%m%y")
        time_string = hst_time.strftime("%H%M%S")
        return date_string + self.delim + time_string


class RobotXClient:
    """
    Handles communication with Technical Director server
    """

    def __init__(self, tcp_ip, tcp_port):
        while True:
            try:
                self.tcp_ip = socket.gethostbyname(tcp_ip)
                break
            except socket.gaierror as e:
                rospy.logwarn("Failed to resolved {}: {}".format(tcp_ip, e))
                rospy.sleep(1.0)
                continue
        self.tcp_port = tcp_port
        self.connected = False
        self.socket_connection = None

    def connect(self):
        if not self.connected:
            rospy.loginfo("Attempting Connection to TD Server at {}:{}".format(self.tcp_ip, self.tcp_port))
        while not self.connected and not rospy.is_shutdown():
            # recreate socket
            self.socket_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # attempt to reconnect, otherwise sleep for 2 seconds
            try:
                self.socket_connection.connect((self.tcp_ip, self.tcp_port))
                self.connected = True
                rospy.loginfo("Connection to TD Server Successful")
            except socket.error:
                rospy.sleep(2)

    @thread_lock(lock)
    def send_message(self, message):
        while not rospy.is_shutdown():
            try:
                self.socket_connection.send(message)
                break
            except socket.error:
                rospy.loginfo("Connection to TD Server Lost")
                self.connected = False
                self.connect()


if __name__ == "__main__":
    robotx_start_services = RobotXStartServices()
    rospy.spin()
