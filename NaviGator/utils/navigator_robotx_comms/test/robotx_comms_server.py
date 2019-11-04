#!/usr/bin/env python

"""
RobotX Communications: A node that handles unit tests for the the RobotX Communication Protocol
"""

import socket
import traceback
import unittest
import threading

import rospy
import rostest
from mil_tools import thread_lock
from navigator_msgs.msg import ScanTheCode
from navigator_msgs.srv import MessageExtranceExitGate, MessageIdentifySymbolsDock, MessageDetectDeliver

from navigator_robotx_comms.navigator_robotx_comms import BitwiseXORChecksum, RobotXDetectDeliverMessage, \
    RobotXHeartbeatMessage, RobotXEntranceExitGateMessage, RobotXScanCodeMessage, RobotXIdentifySymbolsDockMessage

lock = threading.Lock()


class TestRobotXComms(unittest.TestCase):

    def __init__(self, *args):
        # define delimiter for messages
        self.delim = ','
        self.team_id = rospy.get_param("~team_id")
        self.td_ip = rospy.get_param("~td_ip")
        self.td_port = rospy.get_param("~td_port")
        self.number_of_iterations = rospy.get_param("~number_of_iterations")
        self.use_test_data = rospy.get_param("~use_test_data")
        self.server = RobotXServer(self.td_ip, self.td_port)
        self.scan_code_pub = rospy.Publisher('/scan_the_code', ScanTheCode, queue_size=10)
        super(TestRobotXComms, self).__init__(*args)

    def test_heartbeat_message(self):
        times_ran = 0
        self.server.connect()

        robotx_heartbeat_message = RobotXHeartbeatMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robotx_heartbeat_message.from_string(self.delim, message)
                    data_list = deserialized_msg[0]
                    checksum_list = deserialized_msg[1]
                    if data_list[0] == "$RXHRB":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                        hex_checksum = format(tot_checksum, '02X')
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEquals(len(data_list), 10, "heartbeat message formatting incorrect")
                        if self.use_test_data is True:
                            test_data = "$RXHRB,101218,161229,21.31198,N,157.88972,W,AUVSI,2,1*06\r\n"
                            list_test_data = test_data.split(self.delim)
                            checksum_list_test_data = test_data.split("*")
                            self.assertEquals(data_list[7], list_test_data[7], "team id incorrect")
                            self.assertEquals(checksum_list[1], checksum_list_test_data[1],
                                              "heartbeat message checksum incorrect")
                        else:
                            self.assertEquals(data_list[7], self.team_id, "team id incorrect")
                            self.assertEquals(checksum_list[1], final_checksum_string,
                                              "heartbeat message checksum incorrect")
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_entrance_exit_gate_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        entrance_gate = 1
        exit_gate = 2
        light_buoy_active = True
        light_pattern = "RBG"

        rospy.wait_for_service("entrance_exit_gate_message")
        send_robot_x_entrance_exit_gate_message = rospy.ServiceProxy("entrance_exit_gate_message",
                                                                     MessageExtranceExitGate)

        robot_x_entrance_exit_gate_message = RobotXEntranceExitGateMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_entrance_exit_gate_message(entrance_gate, exit_gate, light_buoy_active, light_pattern)
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_entrance_exit_gate_message.from_string(self.delim, message)
                    data_list = deserialized_msg[0]
                    checksum_list = deserialized_msg[1]
                    if data_list[0] == "$RXGAT":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                        hex_checksum = format(tot_checksum, '02X')
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEquals(len(data_list), 8, "entrance exit gate message formatting incorrect")
                        if self.use_test_data is True:
                            test_data = "$RXGAT,101218,161229,AUVSI,1,2,Y,RBG*25\r\n"
                            list_test_data = test_data.split(self.delim)
                            checksum_list_test_data = test_data.split("*")
                            self.assertEquals(data_list[3], list_test_data[3], "team id incorrect")
                            self.assertEquals(checksum_list[1], checksum_list_test_data[1],
                                              "entrance exit gate message checksum incorrect")
                            self.assertEquals(int(data_list[4]), int(list_test_data[4]), "entrance gate incorrect")
                            self.assertEquals(int(data_list[5]), int(list_test_data[5]), "exit gate incorrect")
                            self.assertEquals(data_list[6], list_test_data[6], "light buoy boolean incorrect")
                            msg_light_pattern = data_list[7].split("*")[0]
                            light_pattern = list_test_data[7].split("*")[0]
                            self.assertEquals(msg_light_pattern, light_pattern, "light pattern incorrect")
                        else:
                            self.assertEquals(data_list[3], self.team_id, "team id incorrect")
                            self.assertEquals(checksum_list[1], final_checksum_string,
                                              "entrance exit gate message checksum incorrect")
                            self.assertEquals(int(data_list[4]), entrance_gate, "entrance gate incorrect")
                            self.assertEquals(int(data_list[5]), exit_gate, "exit gate incorrect")
                            if light_buoy_active:
                                self.assertEquals("Y", data_list[6], "light buoy boolean incorrect")
                            else:
                                self.assertEquals("N", data_list[6], "light buoy boolean incorrect")
                            msg_light_pattern = data_list[7].split("*")[0]
                            self.assertEquals(msg_light_pattern, light_pattern, "light pattern incorrect")
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_scan_code_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        color_pattern = "RBG"

        scan_code_msg = ScanTheCode()
        scan_code_msg.color_pattern = color_pattern

        robot_x_scan_code_message = RobotXScanCodeMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                self.scan_code_pub.publish(scan_code_msg)
                rx_data = None
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_scan_code_message.from_string(self.delim, message)
                    data_list = deserialized_msg[0]
                    checksum_list = deserialized_msg[1]
                    if data_list[0] == "$RXCOD":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                        hex_checksum = format(tot_checksum, '02X')
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEquals(len(data_list), 5, "scan code message formatting incorrect")
                        if self.use_test_data is True:
                            test_data = "$RXCOD,101218,161229,AUVSI,RBG*49\r\n"
                            list_test_data = test_data.split(self.delim)
                            checksum_list_test_data = test_data.split("*")
                            self.assertEquals(data_list[3], list_test_data[3], "team id incorrect")
                            self.assertEquals(checksum_list[1], checksum_list_test_data[1],
                                              "scan code message checksum incorrect")
                            msg_color_pattern = data_list[4].split("*")[0]
                            light_pattern = list_test_data[4].split("*")[0]
                            self.assertEquals(msg_color_pattern, light_pattern, "light pattern incorrect")
                        else:
                            self.assertEquals(data_list[3], self.team_id, "team id incorrect")
                            self.assertEquals(checksum_list[1], final_checksum_string,
                                              "scan code message checksum incorrect")
                            msg_color_pattern = data_list[4].split("*")[0]
                            self.assertEquals(msg_color_pattern, color_pattern, "color pattern incorrect")
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_identify_symbols_dock_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        shape_color = "R"
        shape = "TRIAN"

        rospy.wait_for_service("identify_symbols_dock_message")
        send_robot_x_identify_symbols_dock_message = rospy.ServiceProxy("identify_symbols_dock_message",
                                                                        MessageIdentifySymbolsDock)

        robot_x_identify_symbols_dock_message = RobotXIdentifySymbolsDockMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_identify_symbols_dock_message(shape_color, shape)
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_identify_symbols_dock_message.from_string(self.delim, message)
                    data_list = deserialized_msg[0]
                    checksum_list = deserialized_msg[1]
                    if data_list[0] == "$RXDOK":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                        hex_checksum = format(tot_checksum, '02X')
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEquals(len(data_list), 6, "identify symbols dock message formatting incorrect")
                        if self.use_test_data is True:
                            test_data = "$RXDOK,101218,161229,AUVSI,R,TRIAN*28\r\n"
                            list_test_data = test_data.split(self.delim)
                            checksum_list_test_data = test_data.split("*")
                            self.assertEquals(data_list[3], list_test_data[3], "team id incorrect")
                            self.assertEquals(checksum_list[1], checksum_list_test_data[1],
                                              "identify symbols dock message checksum incorrect")
                            self.assertEquals(data_list[4], list_test_data[4], "shape color incorrect")
                            msg_shape = data_list[5].split("*")[0]
                            shape = list_test_data[5].split("*")[0]
                            self.assertEquals(msg_shape, shape, "shape incorrect")
                        else:
                            self.assertEquals(data_list[3], self.team_id, "team id incorrect")
                            self.assertEquals(checksum_list[1], final_checksum_string,
                                              "identify symbols dock message checksum incorrect")
                            self.assertEquals(data_list[4], shape_color, "shape color incorrect")
                            msg_shape = data_list[5].split("*")[0]
                            self.assertEquals(shape, msg_shape, "shape incorrect")
                        times_ran += 1
        finally:
            self.server.disconnect()

    def test_detect_deliver_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        shape_color = "R"
        shape = "CIRCL"

        rospy.wait_for_service("detect_deliver_message")
        send_robot_x_detect_deliver_message = rospy.ServiceProxy("detect_deliver_message",
                                                                 MessageDetectDeliver)

        robot_x_detect_deliver_message = RobotXDetectDeliverMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_detect_deliver_message(shape_color, shape)
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_detect_deliver_message.from_string(self.delim, message)
                    data_list = deserialized_msg[0]
                    checksum_list = deserialized_msg[1]
                    if data_list[0] == "$RXDEL":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                        hex_checksum = format(tot_checksum, '02X')
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEquals(len(data_list), 6, "detect deliver message formatting incorrect")
                        if self.use_test_data is True:
                            test_data = "$RXDEL,101218,161229,AUVSI,R,CIRCL*32\r\n"
                            list_test_data = test_data.split(self.delim)
                            checksum_list_test_data = test_data.split("*")
                            self.assertEquals(data_list[3], list_test_data[3], "team id incorrect")
                            self.assertEquals(checksum_list[1], checksum_list_test_data[1],
                                              "detect deliver message checksum incorrect")
                            self.assertEquals(data_list[4], list_test_data[4], "shape color incorrect")
                            msg_shape = data_list[5].split("*")[0]
                            shape = list_test_data[5].split("*")[0]
                            self.assertEquals(msg_shape, shape, "shape incorrect")
                        else:
                            self.assertEquals(data_list[3], self.team_id, "team id incorrect")
                            self.assertEquals(checksum_list[1], final_checksum_string,
                                              "detect deliver message checksum incorrect")
                            self.assertEquals(data_list[4], shape_color, "shape color incorrect")
                            msg_shape = data_list[5].split("*")[0]
                            self.assertEquals(shape, msg_shape, "shape incorrect")
                        times_ran += 1

        finally:
            self.server.disconnect()


class RobotXServer:
    """
    Handles communication with client for testing
    """

    def __init__(self, tcp_ip, tcp_port):
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.connected = False
        self.socket_connection = None
        self.buffer_size = 1024
        self.conn = None
        self.conn_ip = None

    def connect(self):
        while not self.connected and not rospy.is_shutdown():
            try:
                self.socket_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket_connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket_connection.bind((self.tcp_ip, self.tcp_port))
                self.socket_connection.listen(5)
                self.conn, self.conn_ip = self.socket_connection.accept()
                self.connected = True
            except socket.error:
                traceback.print_exc()
                self.disconnect()
                rospy.sleep(2)

    def disconnect(self):
        if self.conn is not None:
            self.conn.close()
        if self.socket_connection is not None:
            self.socket_connection.close()
        self.connected = False

    @thread_lock(lock)
    def receive_message(self):
        while not rospy.is_shutdown():
            try:
                rx_msg = self.conn.recv(self.buffer_size)
                return rx_msg
            except socket.error:
                self.disconnect()
                self.connect()


if __name__ == "__main__":
    rospy.init_node('robotx_comms_server', anonymous=True)
    rostest.rosrun("robotx_comms", "robotx_comms_server", TestRobotXComms)
    unittest.main()
