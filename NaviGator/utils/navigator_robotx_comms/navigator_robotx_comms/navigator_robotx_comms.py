#!/usr/bin/env python

"""
RobotX Communications Library: A library that handles serialization and deserialization
of messages for the RobotX Communication Protocol
"""

import math

import tf.transformations as trans
from mil_tools import rosmsg_to_numpy
from navigator_msgs.srv import MessageExtranceExitGateResponse, \
    MessageIdentifySymbolsDockResponse, MessageDetectDeliverResponse


class BitwiseXORChecksum:
    def __init__(self):
        self.tot_checksum = 0

    def ret_checksum(self, message):
        for unit_counter in range(len(message)):
            self.tot_checksum = ord(message[unit_counter]) ^ self.tot_checksum
        return self.tot_checksum


class RobotXHeartbeatMessage:
    """
    Handles formation of heartbeat message
    """

    def __init__(self):
        self.message_id = "RXHRB"
        self.timestamp_last = None

    def from_string(self, delim, string):
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(self, delim, team_id, hst_date_time, gps_array, odom, auv_status, system_mode, use_test_data):
        if gps_array is not None:
            latitude = gps_array.point.x
            longitude = gps_array.point.y
        else:
            latitude = ""
            longitude = ""

        if odom is not None:
            quaternion = odom.pose.pose.orientation
            quaternion_numpy = rosmsg_to_numpy(quaternion)
            euler_angles = trans.euler_from_quaternion(quaternion_numpy)
            north_south = ""
            east_west = ""
            if 0 < euler_angles[2] < math.pi / 2:
                north_south = "N"
                east_west = "E"
            elif math.pi / 2 < euler_angles[2] < math.pi:
                north_south = "N"
                east_west = "W"
            elif -math.pi / 2 > euler_angles[2] > -math.pi:
                north_south = "S"
                east_west = "W"
            elif 0 > euler_angles[2] > -math.pi / 2:
                north_south = "S"
                east_west = "E"
        else:
            east_west = ""
            north_south = ""

        if auv_status is None:
            auv_status = 0

        if system_mode is None:
            system_mode = 0

        first_half_data = "{0}{1}{2}{3}{4}{5}{6}".format(self.message_id,
                                                         delim,
                                                         hst_date_time,
                                                         delim,
                                                         latitude,
                                                         delim,
                                                         north_south)

        second_half_data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(longitude,
                                                                delim,
                                                                east_west,
                                                                delim,
                                                                team_id,
                                                                delim, str(system_mode), delim,
                                                                str(auv_status))

        full_data = first_half_data + delim + second_half_data

        # test data
        if use_test_data:
            full_data = "RXHRB,101218,161229,21.31198,N,157.88972,W,AUVSI,2,1"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(full_data)
        hex_checksum = format(checksum, '02X')

        msg_return = "${0}*{1}\r\n".format(full_data, str(hex_checksum).zfill(2))

        return msg_return


class RobotXEntranceExitGateMessage:
    """
    Handles formation of entrance and exit gates message
    """

    def __init__(self):
        self.message_id = "RXGAT"

    def from_string(self, delim, string):
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(self, delim, team_id, hst_date_time, data, use_test_data):
        if data.light_buoy_active:
            light_buoy_active_letter = "Y"
        else:
            light_buoy_active_letter = "N"

        data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}{9}{10}{11}{12}".format(self.message_id,
                                                                   delim,
                                                                   hst_date_time,
                                                                   delim,
                                                                   team_id,
                                                                   delim,
                                                                   str(data.entrance_gate),
                                                                   delim,
                                                                   str(data.exit_gate),
                                                                   delim,
                                                                   light_buoy_active_letter,
                                                                   delim,
                                                                   data.light_pattern)

        # test data
        if use_test_data:
            data = "RXGAT,101218,161229,AUVSI,1,2,Y,RBG"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, '02X')

        msg_return = "${0}*{1}\r\n".format(data, hex_checksum)

        return MessageExtranceExitGateResponse(msg_return)


class RobotXScanCodeMessage:
    """
    Handles formation and sending of scan the code message
    """

    def __init__(self):
        self.message_id = "RXCOD"

    def from_string(self, delim, string):
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(self, delim, team_id, hst_date_time, color_pattern, use_test_data):
        data = "{0}{1}{2}{3}{4}{5}{6}".format(self.message_id,
                                              delim,
                                              hst_date_time,
                                              delim,
                                              team_id,
                                              delim,
                                              color_pattern)

        # test data
        if use_test_data:
            data = "RXCOD,101218,161229,AUVSI,RBG"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, '02X')

        msg_return = "${0}*{1}\r\n".format(data, hex_checksum)

        return msg_return


class RobotXIdentifySymbolsDockMessage:
    """
    Handles formation of identify symbols and dock message
    """

    def __init__(self):
        self.message_id = "RXDOK"

    def from_string(self, delim, string):
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(self, delim, team_id, hst_date_time, data, use_test_data):
        data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(self.message_id,
                                                    delim,
                                                    hst_date_time,
                                                    delim,
                                                    team_id,
                                                    delim,
                                                    data.shape_color,
                                                    delim,
                                                    data.shape)

        # test data
        if use_test_data:
            data = "RXDOK,101218,161229,AUVSI,R,TRIAN"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, '02X')

        msg_return = "${0}*{1}\r\n".format(data, hex_checksum)

        return MessageIdentifySymbolsDockResponse(msg_return)


class RobotXDetectDeliverMessage:
    """
    Handles formation of detect and deliver message
    """

    def __init__(self):
        self.message_id = "RXDEL"

    def from_string(self, delim, string):
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(self, delim, team_id, hst_date_time, data, use_test_data):
        data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(self.message_id,
                                                    delim,
                                                    hst_date_time,
                                                    delim,
                                                    team_id,
                                                    delim,
                                                    data.shape_color,
                                                    delim,
                                                    data.shape)

        # test data
        if use_test_data:
            data = "RXDEL,101218,161229,AUVSI,R,CIRCL"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, '02X')

        msg_return = "${0}*{1}\r\n".format(data, hex_checksum)

        return MessageDetectDeliverResponse(msg_return)
