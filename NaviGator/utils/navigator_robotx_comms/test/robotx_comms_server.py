#!/usr/bin/env python3

"""
RobotX Communications: A node that handles unit tests for the the RobotX Communication Protocol
"""

import socket
import threading
import traceback
import unittest

import rospy
import rostest
from mil_tools import thread_lock
from navigator_msgs.msg import ScanTheCode
from navigator_msgs.srv import (
    MessageDetectDock,
    MessageEntranceExitGate,
    MessageFindFling,
    MessageFollowPath,
    MessageReactReport,
    MessageUAVReplenishment,
    MessageUAVSearchReport,
)
from navigator_robotx_comms.navigator_robotx_comms import (
    BitwiseXORChecksum,
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

lock = threading.Lock()


class TestRobotXComms(unittest.TestCase):
    def __init__(self, *args):
        # define delimiter for messages
        self.delim = b","
        self.team_id = rospy.get_param("~team_id")
        self.td_ip = rospy.get_param("~td_ip")
        self.td_port = rospy.get_param("~td_port")
        self.number_of_iterations = rospy.get_param("~number_of_iterations")
        self.use_test_data = rospy.get_param("~use_test_data")
        self.server = RobotXServer(self.td_ip, self.td_port)
        self.scan_code_pub = rospy.Publisher(
            "/scan_the_code",
            ScanTheCode,
            queue_size=10,
        )
        super().__init__(*args)

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
                    deserialized_msg = robotx_heartbeat_message.from_string(
                        self.delim,
                        message,
                    )
                    data_list = [x.decode() for x in deserialized_msg[0]]
                    checksum_list = [x.decode() for x in deserialized_msg[1]]
                    if data_list[0] == "$RXHRB":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(
                            str(full_data_for_checksum),
                        )
                        hex_checksum = format(tot_checksum, "02X")
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEqual(
                            len(data_list),
                            10,
                            "heartbeat message formatting incorrect",
                        )
                        if self.use_test_data is True:
                            test_data = "$RXHRB,111221,161229,21.31198,N,157.88972,W,ROBOT,2,1*11\r\n"
                            list_test_data = test_data.split(self.delim.decode())
                            checksum_list_test_data = test_data.split("*")
                            self.assertEqual(
                                data_list[7],
                                list_test_data[7],
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                checksum_list_test_data[1],
                                "heartbeat message checksum incorrect",
                            )
                        else:
                            self.assertEqual(
                                data_list[7],
                                self.team_id,
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                final_checksum_string,
                                "heartbeat message checksum incorrect",
                            )
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_entrance_exit_gate_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        entrance_gate = 1
        exit_gate = 2

        rospy.wait_for_service("entrance_exit_gate_message")
        send_robot_x_entrance_exit_gate_message = rospy.ServiceProxy(
            "entrance_exit_gate_message",
            MessageEntranceExitGate,
        )

        robot_x_entrance_exit_gate_message = RobotXEntranceExitGateMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_entrance_exit_gate_message(entrance_gate, exit_gate)
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)

                for message in split_rx_data:
                    deserialized_msg = robot_x_entrance_exit_gate_message.from_string(
                        self.delim,
                        message,
                    )
                    data_list = [x.decode() for x in deserialized_msg[0]]
                    checksum_list = [x.decode() for x in deserialized_msg[1]]
                    if data_list[0] == "$RXGAT":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(
                            full_data_for_checksum,
                        )
                        hex_checksum = format(tot_checksum, "02X")
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEqual(
                            len(data_list),
                            6,
                            "entrance exit gate message formatting incorrect",
                        )
                        if self.use_test_data is True:
                            test_data = "$RXGAT,111221,161229,ROBOT,1,2*3C\r\n"
                            list_test_data = test_data.split(self.delim.decode())
                            checksum_list_test_data = test_data.split("*")
                            self.assertEqual(
                                data_list[3],
                                list_test_data[3],
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                checksum_list_test_data[1],
                                "entrance exit gate message checksum incorrect",
                            )
                            self.assertEqual(
                                int(data_list[4]),
                                int(list_test_data[4]),
                                "entrance gate incorrect",
                            )
                            exit_gate_value = data_list[5].split("*")[0]
                            exit_gate_value_ = list_test_data[5].split("*")[0]
                            self.assertEqual(
                                int(exit_gate_value),
                                int(exit_gate_value_),
                                "exit gate incorrect",
                            )
                        else:
                            self.assertEqual(
                                data_list[3],
                                self.team_id,
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                final_checksum_string,
                                "entrance exit gate message checksum incorrect",
                            )
                            self.assertEqual(
                                int(data_list[4]),
                                entrance_gate,
                                "entrance gate incorrect",
                            )
                            exit_gate_value = data_list[5].split("*")[0]
                            self.assertEqual(
                                int(exit_gate_value),
                                exit_gate,
                                "exit gate incorrect",
                            )
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
                    deserialized_msg = robot_x_scan_code_message.from_string(
                        self.delim,
                        message,
                    )
                    data_list = [x.decode() for x in deserialized_msg[0]]
                    checksum_list = [x.decode() for x in deserialized_msg[1]]
                    if data_list[0] == "$RXCOD":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(
                            full_data_for_checksum,
                        )
                        hex_checksum = format(tot_checksum, "02X")
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEqual(
                            len(data_list),
                            5,
                            "scan code message formatting incorrect",
                        )
                        if self.use_test_data is True:
                            test_data = "$RXCOD,111221,161229,ROBOT,RBG*5E\r\n"
                            list_test_data = test_data.split(self.delim.decode())
                            checksum_list_test_data = test_data.split("*")
                            self.assertEqual(
                                data_list[3],
                                list_test_data[3],
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                checksum_list_test_data[1],
                                "scan code message checksum incorrect",
                            )
                            msg_color_pattern = data_list[4].split("*")[0]
                            color_pattern = list_test_data[4].split("*")[0]
                            self.assertEqual(
                                msg_color_pattern,
                                color_pattern,
                                "light pattern incorrect",
                            )
                        else:
                            self.assertEqual(
                                data_list[3],
                                self.team_id,
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                final_checksum_string,
                                "scan code message checksum incorrect",
                            )
                            msg_color_pattern = data_list[4].split("*")[0]
                            self.assertEqual(
                                msg_color_pattern,
                                color_pattern,
                                "color pattern incorrect",
                            )
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_detect_dock_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        dock_color = "R"
        ams_status = 1

        rospy.wait_for_service("detect_dock_message")
        send_robot_x_detect_dock_message = rospy.ServiceProxy(
            "detect_dock_message",
            MessageDetectDock,
        )

        robot_x_detect_dock_message = RobotXDetectDockMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_detect_dock_message(dock_color, ams_status)
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_detect_dock_message.from_string(
                        self.delim,
                        message,
                    )
                    data_list = [x.decode() for x in deserialized_msg[0]]
                    checksum_list = [x.decode() for x in deserialized_msg[1]]
                    if data_list[0] == "$RXDOK":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(
                            full_data_for_checksum,
                        )
                        hex_checksum = format(tot_checksum, "02X")
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEqual(
                            len(data_list),
                            6,
                            "detect dock message formatting incorrect",
                        )
                        if self.use_test_data is True:
                            test_data = "$RXDOK,111221,161229,ROBOT,R,1*4E\r\n"
                            list_test_data = test_data.split(self.delim.decode())
                            checksum_list_test_data = test_data.split("*")
                            self.assertEqual(
                                data_list[3],
                                list_test_data[3],
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                checksum_list_test_data[1],
                                "detect dock message checksum incorrect",
                            )
                            self.assertEqual(
                                data_list[4],
                                list_test_data[4],
                                "dock color incorrect",
                            )
                            ams_status_ = data_list[5].split("*")[0]
                            msg_ams_status = list_test_data[5].split("*")[0]
                            self.assertEqual(
                                msg_ams_status,
                                ams_status_,
                                "ams status incorrect",
                            )
                        else:
                            self.assertEqual(
                                data_list[3],
                                self.team_id,
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                final_checksum_string,
                                "detect dock message checksum incorrect",
                            )
                            self.assertEqual(
                                data_list[4],
                                dock_color,
                                "dock color incorrect",
                            )
                            ams_status_ = int(data_list[5].split("*")[0])
                            self.assertEqual(
                                ams_status,
                                ams_status_,
                                "ams status incorrect",
                            )
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_follow_path_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        finished = 1

        rospy.wait_for_service("follow_path_message")
        send_robot_x_follow_path_message = rospy.ServiceProxy(
            "follow_path_message",
            MessageFollowPath,
        )

        robot_x_follow_path_message = RobotXFollowPathMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_follow_path_message(finished)
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_follow_path_message.from_string(
                        self.delim,
                        message,
                    )
                    data_list = [x.decode() for x in deserialized_msg[0]]
                    checksum_list = [x.decode() for x in deserialized_msg[1]]
                    if data_list[0] == "$RXPTH":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(
                            full_data_for_checksum,
                        )
                        hex_checksum = format(tot_checksum, "02X")
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEqual(
                            len(data_list),
                            5,
                            "follow path message formatting incorrect",
                        )
                        if self.use_test_data is True:
                            test_data = "$RXPTH,111221,161229,ROBOT,1*3C\r\n"
                            list_test_data = test_data.split(self.delim.decode())
                            checksum_list_test_data = test_data.split("*")
                            self.assertEqual(
                                data_list[3],
                                list_test_data[3],
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                checksum_list_test_data[1],
                                "follow path message checksum incorrect",
                            )
                            msg_finished = data_list[4].split("*")[0]
                            finished_ = list_test_data[4].split("*")[0]
                            self.assertEqual(
                                msg_finished,
                                finished_,
                                "finished status incorrect",
                            )
                        else:
                            self.assertEqual(
                                data_list[3],
                                self.team_id,
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                final_checksum_string,
                                "follow path message checksum incorrect",
                            )
                            finished_ = int(data_list[4].split("*")[0])
                            self.assertEqual(
                                finished,
                                finished_,
                                "finished status incorrect",
                            )
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_react_report_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        animal_array = ["P", "C", "T"]

        rospy.wait_for_service("react_report_message")
        send_robot_x_react_report_message = rospy.ServiceProxy(
            "react_report_message",
            MessageReactReport,
        )

        robot_x_react_report_message = RobotXReactReportMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_react_report_message(animal_array)
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_react_report_message.from_string(
                        self.delim,
                        message,
                    )
                    data_list = [x.decode() for x in deserialized_msg[0]]
                    checksum_list = [x.decode() for x in deserialized_msg[1]]
                    if data_list[0] == "$RXENC":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(
                            full_data_for_checksum,
                        )
                        hex_checksum = format(tot_checksum, "02X")
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEqual(
                            len(data_list),
                            8,
                            "react report message formatting incorrect",
                        )
                        if self.use_test_data is True:
                            test_data = "$RXENC,111221,161229,ROBOT,3,P,C,T*51\r\n"
                            list_test_data = test_data.split(self.delim.decode())
                            checksum_list_test_data = test_data.split("*")
                            self.assertEqual(
                                data_list[3],
                                list_test_data[3],
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                checksum_list_test_data[1],
                                "react report message checksum incorrect",
                            )

                            self.assertEqual(
                                data_list[4],
                                list_test_data[4],
                                "animal array length incorrect",
                            )

                            for i in range(int(data_list[4])):
                                if i != int(data_list[4]) - 1:
                                    self.assertEqual(
                                        data_list[5 + i],
                                        list_test_data[5 + i],
                                        "animal incorrect",
                                    )
                                else:
                                    msg_animal = data_list[5 + i].split("*")[0]
                                    animal_ = list_test_data[5 + i].split("*")[0]
                                    self.assertEqual(
                                        msg_animal,
                                        animal_,
                                        "animal incorrect",
                                    )

                        else:
                            self.assertEqual(
                                data_list[3],
                                self.team_id,
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                final_checksum_string,
                                "react report message checksum incorrect",
                            )
                            self.assertEqual(
                                int(data_list[4]),
                                len(animal_array),
                                "animal array length incorrect",
                            )

                            for i, animal in enumerate(animal_array):
                                if i != len(animal_array) - 1:
                                    self.assertEqual(
                                        data_list[5 + i],
                                        animal,
                                        "animal incorrect",
                                    )
                                else:
                                    animal_ = data_list[5 + i].split("*")[0]
                                    self.assertEqual(
                                        animal,
                                        animal_,
                                        "animal incorrect",
                                    )
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_find_fling_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        color = "R"
        ams_status = 1

        rospy.wait_for_service("find_fling_message")
        send_robot_x_find_fling_message = rospy.ServiceProxy(
            "find_fling_message",
            MessageFindFling,
        )

        robot_x_find_fling_message = RobotXFindFlingMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_find_fling_message(color, ams_status)
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_find_fling_message.from_string(
                        self.delim,
                        message,
                    )
                    data_list = [x.decode() for x in deserialized_msg[0]]
                    checksum_list = [x.decode() for x in deserialized_msg[1]]
                    if data_list[0] == "$RXFLG":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(
                            full_data_for_checksum,
                        )
                        hex_checksum = format(tot_checksum, "02X")
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEqual(
                            len(data_list),
                            6,
                            "find fling message formatting incorrect",
                        )
                        if self.use_test_data is True:
                            test_data = "$RXFLG,111221,161229,ROBOT,R,2*40\r\n"
                            list_test_data = test_data.split(self.delim.decode())
                            checksum_list_test_data = test_data.split("*")
                            self.assertEqual(
                                data_list[3],
                                list_test_data[3],
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                checksum_list_test_data[1],
                                "find fling message checksum incorrect",
                            )
                            self.assertEqual(
                                data_list[4],
                                list_test_data[4],
                                "shape color incorrect",
                            )
                            msg_ams_status = data_list[5].split("*")[0]
                            ams_status_ = list_test_data[5].split("*")[0]
                            self.assertEqual(
                                msg_ams_status,
                                ams_status_,
                                "shape incorrect",
                            )
                        else:
                            self.assertEqual(
                                data_list[3],
                                self.team_id,
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                final_checksum_string,
                                "find fling message checksum incorrect",
                            )
                            self.assertEqual(data_list[4], color, "color incorrect")
                            ams_status_ = int(data_list[5].split("*")[0])
                            self.assertEqual(
                                ams_status,
                                ams_status_,
                                "ams_status status incorrect",
                            )
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_uav_replenishment_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        uav_status = 1
        item_status = 0

        rospy.wait_for_service("uav_replenishment_message")
        send_robot_x_uav_replenishment_message = rospy.ServiceProxy(
            "uav_replenishment_message",
            MessageUAVReplenishment,
        )

        robot_x_uav_replenishment_message = RobotXUAVReplenishmentMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_uav_replenishment_message(uav_status, item_status)
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_uav_replenishment_message.from_string(
                        self.delim,
                        message,
                    )
                    data_list = [x.decode() for x in deserialized_msg[0]]
                    checksum_list = [x.decode() for x in deserialized_msg[1]]
                    if data_list[0] == "$RXUAV":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(
                            full_data_for_checksum,
                        )
                        hex_checksum = format(tot_checksum, "02X")
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEqual(
                            len(data_list),
                            6,
                            "uav replenishment message formatting incorrect",
                        )
                        if self.use_test_data is True:
                            test_data = "$RXUAV,111221,161229,ROBOT,2,1*2C\r\n"
                            list_test_data = test_data.split(self.delim.decode())
                            checksum_list_test_data = test_data.split("*")
                            self.assertEqual(
                                data_list[3],
                                list_test_data[3],
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                checksum_list_test_data[1],
                                "uav replenishment message checksum incorrect",
                            )
                            self.assertEqual(
                                data_list[4],
                                list_test_data[4],
                                "uav status incorrect",
                            )
                            msg_item_status = data_list[5].split("*")[0]
                            item_status_ = list_test_data[5].split("*")[0]
                            self.assertEqual(
                                msg_item_status,
                                item_status_,
                                "item_status incorrect",
                            )
                        else:
                            self.assertEqual(
                                data_list[3],
                                self.team_id,
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                final_checksum_string,
                                "uav replenishment message checksum incorrect",
                            )
                            self.assertEqual(
                                int(data_list[4]),
                                uav_status,
                                "uav_status incorrect",
                            )
                            item_status_ = int(data_list[5].split("*")[0])
                            self.assertEqual(
                                item_status,
                                item_status_,
                                "finished status incorrect",
                            )
                        times_ran += 1

        finally:
            self.server.disconnect()

    def test_uav_search_report_message(self):
        times_ran = 0
        self.server.connect()
        # data to test message with
        object1 = "R"
        object2 = "S"
        uav_status = 2

        rospy.wait_for_service("uav_search_report_message")
        send_robot_x_uav_search_report_message = rospy.ServiceProxy(
            "uav_search_report_message",
            MessageUAVSearchReport,
        )

        robot_x_uav_search_report_message = RobotXUAVSearchReportMessage()

        try:
            while not rospy.is_shutdown() and times_ran < self.number_of_iterations:
                rx_data = None
                send_robot_x_uav_search_report_message(
                    object1,
                    0,
                    "",
                    0,
                    "",
                    object2,
                    0,
                    "",
                    0,
                    "",
                    uav_status,
                )
                while rx_data is None:
                    rx_data = self.server.receive_message()
                split_rx_data = rx_data.splitlines(True)
                for message in split_rx_data:
                    deserialized_msg = robot_x_uav_search_report_message.from_string(
                        self.delim,
                        message,
                    )
                    data_list = [x.decode() for x in deserialized_msg[0]]
                    checksum_list = [x.decode() for x in deserialized_msg[1]]
                    if data_list[0] == "$RXSAR":
                        full_data_for_checksum = checksum_list[0].replace("$", "")
                        checksum_calc = BitwiseXORChecksum()
                        tot_checksum = checksum_calc.ret_checksum(
                            full_data_for_checksum,
                        )
                        hex_checksum = format(tot_checksum, "02X")
                        final_checksum_string = hex_checksum + "\r\n"
                        self.assertEqual(
                            len(data_list),
                            15,
                            "follow path message formatting incorrect",
                        )
                        if self.use_test_data is True:
                            test_data = "$RXSAR,111221,161229,R,21.31198,N,157.88972,W,N, 21.32198,N,157.89972,W,ROBOT,2*0D\r\n"
                            list_test_data = test_data.split(self.delim.decode())
                            checksum_list_test_data = test_data.split("*")
                            self.assertEqual(
                                data_list[13],
                                list_test_data[13],
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                checksum_list_test_data[1],
                                "follow path message checksum incorrect",
                            )
                            self.assertEqual(
                                data_list[3],
                                list_test_data[3],
                                "object1 incorrect",
                            )
                            self.assertEqual(
                                data_list[8],
                                list_test_data[8],
                                "object2 incorrect",
                            )
                            msg_uav_status = data_list[14].split("*")[0]
                            uav_status_ = list_test_data[14].split("*")[0]
                            self.assertEqual(
                                msg_uav_status,
                                uav_status_,
                                "uav_status incorrect",
                            )
                        else:
                            self.assertEqual(
                                data_list[13],
                                self.team_id,
                                "team id incorrect",
                            )
                            self.assertEqual(
                                checksum_list[1],
                                final_checksum_string,
                                "follow path message checksum incorrect",
                            )
                            self.assertEqual(
                                data_list[3],
                                object1,
                                "object 1 incorrect",
                            )
                            self.assertEqual(
                                data_list[8],
                                object2,
                                "object 2 incorrect",
                            )
                            uav_status_ = int(data_list[14].split("*")[0])
                            self.assertEqual(
                                uav_status,
                                uav_status_,
                                "uav status incorrect",
                            )
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
                self.socket_connection = socket.socket(
                    socket.AF_INET,
                    socket.SOCK_STREAM,
                )
                self.socket_connection.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_REUSEADDR,
                    1,
                )
                self.socket_connection.bind((self.tcp_ip, self.tcp_port))
                self.socket_connection.listen(5)
                self.conn, self.conn_ip = self.socket_connection.accept()
                self.connected = True
            except OSError:
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
            except OSError:
                self.disconnect()
                self.connect()


if __name__ == "__main__":
    rospy.init_node("robotx_comms_server", anonymous=True)
    rostest.rosrun("navigator_robotx_comms", "robotx_comms_server", TestRobotXComms)
    unittest.main()
