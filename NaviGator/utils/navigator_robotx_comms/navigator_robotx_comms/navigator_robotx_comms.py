#!/usr/bin/env python3
"""
RobotX Communications Library: A library that handles serialization and deserialization
of messages for the RobotX Communication Protocol
"""

import math

import tf.transformations as trans
from mil_tools import rosmsg_to_numpy
from navigator_msgs.srv import (
    MessageExtranceExitGateRequest,
    MessageExtranceExitGateResponse,
    MessageIdentifySymbolsDockResponse,
    MessageDetectDeliverRequest,
    MessageDetectDeliverResponse,
)
from nav_msgs.msg import Odometry

from typing import List, Tuple, Optional, Any


class BitwiseXORChecksum:
    def __init__(self):
        self.tot_checksum = 0

    def ret_checksum(self, message):
        for unit_counter in range(len(message)):
            self.tot_checksum = ord(message[unit_counter]) ^ self.tot_checksum
        return self.tot_checksum


class RobotXHeartbeatMessage:
    """
    Handles formation of the heartbeat message.

    .. warning::

        The following code pertains to the **2018 edition** of the AUSVI RobotX
        competition, held in Hawaii. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID identifying the message as a MIL heartbeat message.
            For the 2018 season, this was ``RXHRB``.
    """

    def __init__(self):
        self.message_id = "RXHRB"
        self.timestamp_last = None

    def from_string(self, delim: str, string: str) -> Tuple[List[str], List[str]]:
        """
        From a message represeting a message as a string, return the data and checksum
        lists encoded in the string.

        Args:
            delim (str): The delimeter separating the values in the data list. Frequently
                is a comma.
            string (str): The message represented as a string.

        Returns:
            Tuple[List[str], List[str]]: A tuple, where the first element is a list
            of data values, and the second element is the checksum info.
        """
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: str,
        hst_date_time: Any,
        gps_array: Optional[Any],
        odom: Optional[Odometry],
        auv_status: Optional[int],
        system_mode: Optional[int],
        use_test_data: bool,
    ) -> str:
        """
        Given the necessary information to encode in the message, a message (as a string)
        is created. This message is ready to be sent back through the RobotX communications
        protcol.

        Args:
            delim (str): The delimeter to use when separating the data.
            team_id (str): The team ID used by MIL when sending messages.
            hst_date_time (Any): Presumably (??) a datetime object representing the
                current time in HST.
            gps_array (Optional[Any]): A specific message type containing at least a point. (??)
            odom (Optional[Odometry]): An optional odometry message to encode in the message.
                If ``None``, then empty strings are used in the message instead of the
                current position.
            auv_status (Optional[int]): The status of the AUV in the water. If ``None``,
                then zero is used in the message.
            system_mode (Optional[int]): The current mode of the AUV. If ``None``,
                then zero is used in the message.
            use_test_data (bool): Whether to use a sample message. If so, most of the
                other parameters are ignored, as they are not needed.

        Returns:
            str: The constructed message.
        """
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

        first_half_data = "{0}{1}{2}{3}{4}{5}{6}".format(
            self.message_id, delim, hst_date_time, delim, latitude, delim, north_south
        )

        second_half_data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(
            longitude,
            delim,
            east_west,
            delim,
            team_id,
            delim,
            str(system_mode),
            delim,
            str(auv_status),
        )

        full_data = first_half_data + delim + second_half_data

        # test data
        if use_test_data:
            full_data = "RXHRB,101218,161229,21.31198,N,157.88972,W,AUVSI,2,1"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(full_data)
        hex_checksum = format(checksum, "02X")

        msg_return = "${0}*{1}\r\n".format(full_data, str(hex_checksum).zfill(2))

        return msg_return


class RobotXEntranceExitGateMessage:
    """
    Handles formation of entrance and exit gates message.

    .. warning::

        The following code pertains to the **2018 edition** of the AUSVI RobotX
        competition, held in Hawaii. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message to signal that it is related to the
            entrance and exit gates.
    """

    def __init__(self):
        self.message_id = "RXGAT"

    def from_string(self, delim: str, string: str) -> Tuple[List[str], List[str]]:
        """
        Constructs a list of data values and a checksum list from a provided message.

        Args:
            delim (str): The delimeter splitting up the data values.
            string (str): The message to get the data values from.

        Returns:
            Tuple[List[str], List[str]]: A tuple representing two values. The first
            is the list of data values encoded in the message. The second value
            is the checksum list encoded in the message.
        """
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        hst_date_time: Any,
        data: MessageExtranceExitGateRequest,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a message using the provided parameters. This message is formatted
        according to 2018 AUSVI specifications.

        Args:
            delim (str): The delimeter to use in between data values.
            team_id (Any): A value (??) that can be converted to a string to represent
                the MIL team ID.
            hst_date_time (Any): A value (??) used to represent the current date + time
                in HST.
            data (MessageDetectDeliverRequest): The data about the entrance/exit
                gate mission.
            use_test_data (bool): Whether to use test data in the message. If ``True``,
                then most of the other parameters are ignored.

        Returns:
            str: The encoded message.
        """
        if data.light_buoy_active:
            light_buoy_active_letter = "Y"
        else:
            light_buoy_active_letter = "N"

        data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}{9}{10}{11}{12}".format(
            self.message_id,
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
            data.light_pattern,
        )

        # test data
        if use_test_data:
            data = "RXGAT,101218,161229,AUVSI,1,2,Y,RBG"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = "${0}*{1}\r\n".format(data, hex_checksum)

        return MessageExtranceExitGateResponse(msg_return)


class RobotXScanCodeMessage:
    """
    Handles formation and sending of scan the code message.

    .. warning::

        The following code pertains to the **2018 edition** of the AUSVI RobotX
        competition, held in Hawaii. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID identifying the message as a Scan the Code message type.
            Equal to ``RXCOD``.
    """

    def __init__(self):
        self.message_id = "RXCOD"

    def from_string(self, delim: str, string: str) -> Tuple[List[str], List[str]]:
        """
        Returns the information encoded in a message.

        Args:
            delim (str): The delimeter separating values in the data list encoded
                in the message.
            string (str): The message string to use to get data from.

        Returns:
            Tuple[List[str], List[str]]: The tuple containing a list of data values
            as the first value, and a list of checksums as the second value.
        """
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        hst_date_time: Any,
        color_pattern: str,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a Scan the Code status message.

        Args:
            delim (str): The string delimeter used to separate distinct data
                points in the message.
            team_id (Any): The team ID used by MIL in the competition.
            hst_date_time (Any): The datetime to send in HST.
            color_pattern (str): The color pattern to send in the message.
            use_test_data (bool): Whether to use test data when sending the message.

        Returns:
            str: The constructed message.
        """
        data = "{0}{1}{2}{3}{4}{5}{6}".format(
            self.message_id, delim, hst_date_time, delim, team_id, delim, color_pattern
        )

        # test data
        if use_test_data:
            data = "RXCOD,101218,161229,AUVSI,RBG"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = "${0}*{1}\r\n".format(data, hex_checksum)

        return msg_return


class RobotXIdentifySymbolsDockMessage:
    """
    Handles formation of identify symbols and dock message

    .. warning::

        The following code pertains to the **2018 edition** of the AUSVI RobotX
        competition, held in Hawaii. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message indicating that the message is a
            Symbols and Dock message. Equal to ``RXDOK``.
    """

    def __init__(self):
        self.message_id = "RXDOK"

    def from_string(self, delim: str, string: str) -> Tuple[List[str], List[str]]:
        """
        Retrieves the data and checksum list from a message.

        Args:
            delim (str): The delimeter used to separate distinct datapoints.
            string (str): The source message, as a string.

        Returns:
            Tuple[List[str], List[str]]: A tuple containing the list of data values
            encoded in the message, along with the list of checksums in the message.
        """
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        hst_date_time: Any,
        data: MessageDetectDeliverRequest,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a status message with the provided information.

        Args:
            delim (str): The delimeter used to separate distinct data values that
                need to be sent.
            team_id (Any): The team ID used by MIL at the competition.
            hst_date_time (Any): A value (??) used to represent the date/time combination
                in HST.
            data (MessageDetectDeliverRequest): The message request holding data
                about the mission.
            use_test_data (bool): Whether to use test data when constructing the message.
        """
        data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(
            self.message_id,
            delim,
            hst_date_time,
            delim,
            team_id,
            delim,
            data.shape_color,
            delim,
            data.shape,
        )

        # test data
        if use_test_data:
            data = "RXDOK,101218,161229,AUVSI,R,TRIAN"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = "${0}*{1}\r\n".format(data, hex_checksum)

        return MessageIdentifySymbolsDockResponse(msg_return)


class RobotXDetectDeliverMessage:
    """
    Handles formation of a Detect and Deliver Message to send to AUSVI.

    .. warning::

        The following code pertains to the **2018 edition** of the AUSVI RobotX
        competition, held in Hawaii. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message indicating that is a Detect and Deliver
            Message. Equal to ``RXDEL``.
    """

    def __init__(self):
        self.message_id = "RXDEL"

    def from_string(self, delim: str, string: str) -> Tuple[List[str], List[str]]:
        """
        Retrieves the list of data values and checksums found in one instance of
        a constructed message.

        Args:
            delim (str): The delimeter separating the distinct data values.
            string (str): The source message to retrieve the values from.

        Returns:
            Tuple[List[str], List[str]]: A tuple containing the list of data values
            and the list of checksums.
        """
        data_list = string.split(delim)
        checksum_list = string.split("*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        hst_date_time: Any,
        data: MessageDetectDeliverRequest,
        use_test_data: bool,
    ) -> MessageDetectDeliverResponse:
        """
        Constructs a status message given the provided information.

        Args:
            delim (str): The delimeter separating individual data values.
            team_id (Any): The team ID used by MIL in competition.
            hst_date_time (Any): A representation of a particular date/time in HST.
            data (MessageDetectDeliverRequest): The data request associated with the mission.
            use_test_data (bool): Whether to use test data to construct the message.

        Returns:
            MessageDetectDeliverResponse: The constructed message response.
        """
        data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(
            self.message_id,
            delim,
            hst_date_time,
            delim,
            team_id,
            delim,
            data.shape_color,
            delim,
            data.shape,
        )

        # test data
        if use_test_data:
            data = "RXDEL,101218,161229,AUVSI,R,CIRCL"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = "${0}*{1}\r\n".format(data, hex_checksum)

        return MessageDetectDeliverResponse(msg_return)
