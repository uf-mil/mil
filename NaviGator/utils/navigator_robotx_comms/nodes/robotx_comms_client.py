#!/usr/bin/env python3

"""
RobotX Communications: A node that handles message communications
with the Technical Director server for the RobotX Communication Protocol
"""

import datetime
import socket
import threading
from enum import IntEnum

import rospy
from geometry_msgs.msg import PointStamped
from mil_tools import thread_lock
from nav_msgs.msg import Odometry
from navigator_msgs.msg import ScanTheCode
from navigator_msgs.srv import (
    MessageDetectDock,
    MessageDetectDockRequest,
    MessageDetectDockResponse,
    MessageEntranceExitGate,
    MessageEntranceExitGateRequest,
    MessageEntranceExitGateResponse,
    MessageFindFling,
    MessageFindFlingRequest,
    MessageFindFlingResponse,
    MessageFollowPath,
    MessageFollowPathRequest,
    MessageFollowPathResponse,
    MessageReactReport,
    MessageReactReportRequest,
    MessageReactReportResponse,
    MessageUAVReplenishment,
    MessageUAVReplenishmentRequest,
    MessageUAVReplenishmentResponse,
    MessageUAVSearchReport,
    MessageUAVSearchReportRequest,
    MessageUAVSearchReportResponse,
)
from navigator_robotx_comms.navigator_robotx_comms import (
    RobotXDetectDockMessage,
    RobotXEntranceExitGateMessage,
    RobotXFindFlingMessage,
    RobotXFollowPathMessage,
    RobotXHeartbeatMessage,
    RobotXReactReportMessage,
    RobotXScanCodeMessage,
    RobotXUAVReplenishmentMessage,
    RobotXUAVSearchReportMessage,
)
from ros_alarms import AlarmListener
from ros_alarms_msgs.msg import Alarm
from std_msgs.msg import String

lock = threading.Lock()


class SystemModes(IntEnum):
    """
    Enumerates constants of friendly system mode names to ints
    """

    KILLED = 3
    AUTONOMOUS = 2
    REMOTE_CONTROLLED = 1


class RobotXStartServices:
    """
    Initializes services and subscribes to necessary publishers in order to facilitate
    the transmission of messages between the robot client and the Technical Director
    server.

    This class is part of a node that should be launched when communication is necessary
    with official AUVSI software.

    Attributes:
        robotx_client (RobotXClient): The client used to connect with the Technical
            Director server.
        system_mode (int): The mode of the system.
        wrench (str): The current wrench mode of the system. For example, ``autonomous``.
    """

    def __init__(self):
        # define all variables for subscribers
        self.gps_array = None
        self.odom = None
        self.uav_status = 1
        self.system_mode = None
        self.wrench = None
        self.kill = False
        # define delimiter for messages
        self.delim = ","
        # define parameters
        self.team_id = rospy.get_param("~team_id")
        self.td_ip = rospy.get_param("~td_ip")
        self.td_port = rospy.get_param("~td_port")
        self.use_test_data = rospy.get_param("~use_test_data")
        # time last called
        self.time_last_entrance_exit = None
        self.time_last_follow_path = None
        self.time_last_react_report = None
        self.time_last_find_fling = None
        self.time_last_uav_replenishment = None
        self.time_last_uav_search_report = None
        self.time_last_scan_code = None
        self.time_last_detect_dock = None
        # initialize connection to server
        self.robotx_client = RobotXClient(self.td_ip, self.td_port)
        self.robotx_client.connect()

        # setup all message types
        self.robotx_heartbeat_message = RobotXHeartbeatMessage()
        self.robotx_entrance_exit_gate_message = RobotXEntranceExitGateMessage()
        self.robotx_scan_code_message = RobotXScanCodeMessage()
        self.robotx_detect_dock_message = RobotXDetectDockMessage()
        self.robotx_follow_path_message = RobotXFollowPathMessage()
        self.robotx_react_report_message = RobotXReactReportMessage()
        self.robotx_find_fling_message = RobotXFindFlingMessage()
        self.robotx_uav_replenishment_message = RobotXUAVReplenishmentMessage()
        self.robotx_uav_search_report_message = RobotXUAVSearchReportMessage()

        # setup all subscribers
        rospy.Subscriber("lla", PointStamped, self.gps_coord_callback)
        rospy.Subscriber("odom", Odometry, self.gps_odom_callback)
        rospy.Subscriber("/wrench/selected", String, self.wrench_callback)
        rospy.Subscriber("/scan_the_code", ScanTheCode, self.scan_the_code_callback)

        # TODO: setup subscriber for uav status callback update

        # track kill state for inferring system mode
        self.kill_listener = AlarmListener("kill", self.kill_callback)
        self.kill_listener.wait_for_server()

        # setup all services
        self.service_entrance_exit_gate_message = rospy.Service(
            "entrance_exit_gate_message",
            MessageEntranceExitGate,
            self.handle_entrance_exit_gate_message,
        )
        self.service_follow_path_message = rospy.Service(
            "follow_path_message",
            MessageFollowPath,
            self.handle_follow_path_message,
        )
        self.service_react_report_message = rospy.Service(
            "react_report_message",
            MessageReactReport,
            self.handle_react_report_message,
        )
        self.service_detect_dock_message = rospy.Service(
            "detect_dock_message",
            MessageDetectDock,
            self.handle_detect_dock_message,
        )
        self.service_find_fling_message = rospy.Service(
            "find_fling_message",
            MessageFindFling,
            self.handle_find_fling_message,
        )
        self.service_uav_replenishment_message = rospy.Service(
            "uav_replenishment_message",
            MessageUAVReplenishment,
            self.handle_uav_replenishment_message,
        )
        self.service_uav_search_report_message = rospy.Service(
            "uav_search_report_message",
            MessageUAVSearchReport,
            self.handle_uav_search_report_message,
        )

        # start sending heartbeat every second
        rospy.Timer(rospy.Duration(1), self.handle_heartbeat_message)

    def update_system_mode(self) -> None:
        """
        Sets :attr:`.system_mode` according to whether the kill is ``True`` and the mode
        of the boat.
        """
        if self.kill is True:
            self.system_mode = SystemModes.KILLED
        elif self.wrench == "autonomous" or self.wrench == "/wrench/autonomous":
            self.system_mode = SystemModes.AUTONOMOUS
        else:
            self.system_mode = SystemModes.REMOTE_CONTROLLED

    def wrench_callback(self, wrench: String) -> None:
        """
        Updates the :attr:`.wrench` attribute with the most recent wrench value.
        """
        self.wrench = wrench.data

    def kill_callback(self, alarm: Alarm):
        """
        Updates the :attr:`.kill` attribute depending on whether the most recent
        alarm received by the boat was raised or not.
        """
        self.kill = alarm.raised

    def gps_coord_callback(self, lla: PointStamped) -> None:
        """
        Updates the :attr:`.gps_array` attribute with the most recent :class:`PointStamped`
        message received.
        """
        self.gps_array = lla

    def gps_odom_callback(self, odom: Odometry):
        """
        Stores the most recent :class:`Odometry` message.
        """
        self.odom = odom

    def uav_status_callback(self, uav_status: int):
        """
        Stores the most recent AUV status experienced by the boat.
        """
        self.uav_status = uav_status

    def system_mode_callback(self, system_mode: int):
        """
        Sets the class' :attr:`system_mode` attribute when given the most recent
        system mode.
        """
        self.system_mode = system_mode

    def scan_the_code_callback(self, scan_the_code: ScanTheCode):
        """
        Handles the ScanTheCode message requests by calling :meth:`.handle_scan_code_message`
        with the color pattern specified in the message.

        Args:
            scan_the_code (ScanTheCode): The message to format and send to the Technical
                Director server.
        """
        self.handle_scan_code_message(scan_the_code.color_pattern)

    def handle_heartbeat_message(self, _) -> None:
        """
        Constructs a heartbeat message according to a timer and sends the formatted
        message to the AUVSI Technical Director station.
        """
        self.update_system_mode()
        aedt_date_time = self.get_aedt_date_time()

        message = self.robotx_heartbeat_message.to_string(
            self.delim,
            self.team_id,
            aedt_date_time,
            self.gps_array,
            self.odom,
            self.uav_status,
            self.system_mode,
            self.use_test_data,
        )
        self.robotx_client.send_message(message)

    def handle_entrance_exit_gate_message(
        self,
        data: MessageEntranceExitGateRequest,
    ) -> MessageEntranceExitGateResponse:
        """
        Handles requests to make messages to use in the Entrance and Exit Gate
        mission.

        Args:
            data (MessageEntranceExitGateRequest): The request to the service.

        Returns:
            MessageEntranceExitGateResponse: The response from the service. The response
            contains the message needed to send to AUVSI.
        """
        if self.time_last_entrance_exit is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_entrance_exit
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_entrance_exit = rospy.get_time()
        aedt_date_time = self.get_aedt_date_time()
        message = self.robotx_entrance_exit_gate_message.to_string(
            self.delim,
            self.team_id,
            aedt_date_time,
            data,
            self.use_test_data,
        )

        self.robotx_client.send_message(message)
        return MessageEntranceExitGateResponse(message)

    def handle_follow_path_message(
        self,
        data: MessageFollowPathRequest,
    ) -> MessageFollowPathResponse:
        """
        Handles requests to make messages to use in the Follow Path Mission

        Args:
            data (MessageFollowPathRequest): The request to the service.

        Returns:
            MessageFollowPathResponse: The response from the service. The response
            contains the message needed to send to AUVSI.
        """
        if self.time_last_follow_path is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_follow_path
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_follow_path = rospy.get_time()
        aedt_date_time = self.get_aedt_date_time()
        message = self.robotx_follow_path_message.to_string(
            self.delim,
            self.team_id,
            aedt_date_time,
            data,
            self.use_test_data,
        )

        self.robotx_client.send_message(message)
        return MessageFollowPathResponse(message)

    def handle_react_report_message(
        self,
        data: MessageReactReportRequest,
    ) -> MessageReactReportResponse:
        """
        Handles requests to make messages to use in the React Report Mission

        Args:
            data (MessageReactReportRequest): The request to the service.

        Returns:
            MessageReactReportResponse: The response from the service. The response
            contains the message needed to send to AUVSI.
        """
        if self.time_last_react_report is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_react_report
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_react_report = rospy.get_time()
        aedt_date_time = self.get_aedt_date_time()
        message = self.robotx_react_report_message.to_string(
            self.delim,
            self.team_id,
            aedt_date_time,
            data,
            self.use_test_data,
        )

        self.robotx_client.send_message(message)
        return MessageReactReportResponse(message)

    def handle_scan_code_message(self, color_pattern: str) -> None:
        """
        Handles the color pattern reported over the topic dedicated to the Scan the Code
        mission.

        The color is encoded into a message and sent to the AUVSI Technical Director
        server.

        Args:
            color_pattern (str): The color pattern reported by the handled message
                in the Scan the Code topic.
        """
        if self.time_last_scan_code is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_scan_code
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_scan_code = rospy.get_time()
        aedt_date_time = self.get_aedt_date_time()
        message = self.robotx_scan_code_message.to_string(
            self.delim,
            self.team_id,
            aedt_date_time,
            color_pattern,
            self.use_test_data,
        )
        self.robotx_client.send_message(message)

    def handle_detect_dock_message(
        self,
        data: MessageDetectDockRequest,
    ) -> MessageDetectDockResponse:
        """
        Handles requests to make messages to use in the Detect Dock Mission

        Args:
            data (MessageDetectDockRequest): The request to the service.

        Returns:
            MessageDetectDockResponse: The response from the service. The response
            contains the message needed to send to AUVSI.
        """
        if self.time_last_detect_dock is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_detect_dock
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_detect_dock = rospy.get_time()
        aedt_date_time = self.get_aedt_date_time()
        message = self.robotx_detect_dock_message.to_string(
            self.delim,
            self.team_id,
            aedt_date_time,
            data,
            self.use_test_data,
        )

        self.robotx_client.send_message(message)
        return MessageDetectDockResponse(message)

    def handle_find_fling_message(
        self,
        data: MessageFindFlingRequest,
    ) -> MessageFindFlingResponse:
        """
        Handles requests to make messages to use in the Find Fling Mission

        Args:
            data (MessageFindFlingRequest): The request to the service.

        Returns:
            MessageFindFlingResponse: The response from the service. The response
            contains the message needed to send to AUVSI.
        """
        if self.time_last_find_fling is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_find_fling
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_find_fling = rospy.get_time()
        aedt_date_time = self.get_aedt_date_time()
        message = self.robotx_find_fling_message.to_string(
            self.delim,
            self.team_id,
            aedt_date_time,
            data,
            self.use_test_data,
        )

        self.robotx_client.send_message(message)
        return MessageFindFlingResponse(message)

    def handle_uav_replenishment_message(
        self,
        data: MessageUAVReplenishmentRequest,
    ) -> MessageUAVReplenishmentResponse:
        """
        Handles requests to make messages to use in the UAV Replenishment Mission

        Args:
            data (MessageUAVReplenishmentRequest): The request to the service.

        Returns:
            MessageUAVReplenishmentResponse: The response from the service. The response
            contains the message needed to send to AUVSI.
        """
        if self.time_last_uav_replenishment is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_uav_replenishment
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_uav_replenishment = rospy.get_time()
        aedt_date_time = self.get_aedt_date_time()
        message = self.robotx_uav_replenishment_message.to_string(
            self.delim,
            self.team_id,
            aedt_date_time,
            data,
            self.use_test_data,
        )

        self.robotx_client.send_message(message)
        return MessageUAVReplenishmentResponse(message)

    def handle_uav_search_report_message(
        self,
        data: MessageUAVSearchReportRequest,
    ) -> MessageUAVSearchReportResponse:
        """
        Handles requests to make messages to use in the UAV Search Report Mission

        Args:
            data (MessageUAVSearchReportRequest): The request to the service.

        Returns:
            MessageUAVSearchReportResponse: The response from the service. The response
            contains the message needed to send to AUVSI.
        """
        if self.time_last_uav_search_report is not None:
            seconds_elapsed = rospy.get_time() - self.time_last_uav_search_report
            if seconds_elapsed < 1:
                rospy.sleep(1 - seconds_elapsed)
        self.time_last_uav_search_report = rospy.get_time()
        aedt_date_time = self.get_aedt_date_time()
        message = self.robotx_uav_search_report_message.to_string(
            self.delim,
            self.team_id,
            aedt_date_time,
            data,
            self.use_test_data,
        )

        self.robotx_client.send_message(message)
        return MessageUAVSearchReportResponse(message)

    def get_aedt_date_time(self) -> str:
        """
        Gets the current time in AEDT in the format of ``%d%m%y{self.delim}%H%M%S``.
        This is the format specified by AUVSI to use in messages.

        Returns:
            str: The constructed string representing the date and time.
        """
        # AEDT is 11 hours ahead of UTC
        aedt_time = datetime.datetime.utcnow() + datetime.timedelta(hours=11, minutes=0)
        date_string = aedt_time.strftime("%d%m%y")
        time_string = aedt_time.strftime("%H%M%S")
        return date_string + self.delim + time_string


class RobotXClient:
    """
    Handles communication with Technical Director server at the AUVSI RobotX competition.

    This class is generally handled by :class:`RobotXStartServices`.

    Attributes:
        tcp_port (int): The TCP port allotted for communication between MIL and the
            Technical Director server at the competition site.
        connected (bool): Whether a connection is intact between the client and the
            Technical Director server.
        socket_connection (Optional[socket.socket]): The socket connection. This is
            initially set to ``None``, until it is made in :meth:`.connect`.
    """

    def __init__(self, tcp_ip: str, tcp_port: int):
        """
        Args:
            tcp_ip (str): The IP address used by the Technical Director server to connect to.
            tcp_port (int): The port used by the Technical Director server that MIL
                is asked to connect to.
        """
        while True:
            try:
                self.tcp_ip = socket.gethostbyname(tcp_ip)
                break
            except socket.gaierror as e:
                rospy.logwarn(f"Failed to resolved {tcp_ip}: {e}")
                rospy.sleep(1.0)
                continue
        self.tcp_port = tcp_port
        self.connected = False
        self.socket_connection = None

    def connect(self) -> None:
        """
        Attempts to establish a socket connection at the given IP and port number
        if no connection has already been established.

        If a connection can not be established, then the method sleeps for 2 seconds
        before retrying to establish a connection.
        """
        if not self.connected:
            rospy.loginfo(
                "Attempting Connection to TD Server at {}:{}".format(
                    self.tcp_ip,
                    self.tcp_port,
                ),
            )
        while not self.connected and not rospy.is_shutdown():
            # recreate socket
            self.socket_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # attempt to reconnect, otherwise sleep for 2 seconds
            try:
                self.socket_connection.connect((self.tcp_ip, self.tcp_port))
                self.connected = True
                rospy.loginfo("Connection to TD Server Successful")
            except OSError:
                rospy.sleep(2)

    @thread_lock(lock)
    def send_message(self, message: bytes) -> None:
        """
        Sends a message over the established socket connection. This method is thread-locked.

        If any error occurs in the transmission of the packet, then :attr:`.connection`
        is immediately set to ``False``, and a connection is attempted to be re-established
        through :meth:`.connect`.

        Args:
            message (bytes): The message to send over the connection.
        """
        while not rospy.is_shutdown():
            try:
                self.socket_connection.send(bytes(message, "ascii"))
                break
            except OSError:
                rospy.loginfo("Connection to TD Server Lost")
                self.connected = False
                self.connect()


if __name__ == "__main__":
    rospy.init_node("robotx_comms_client")
    robotx_start_services = RobotXStartServices()
    rospy.spin()
