#!/usr/bin/env python3
"""
RobotX Communications Library: A library that handles serialization and deserialization
of messages for the RobotX Communication Protocol
"""

from __future__ import annotations

import math
from typing import Any

import tf.transformations as trans
from geographic_msgs.msg import GeoPoint
from mil_tools import rosmsg_to_numpy
from nav_msgs.msg import Odometry
from navigator_msgs.srv import (
    MessageDetectDockRequest,
    MessageEntranceExitGateRequest,
    MessageFindFlingRequest,
    MessageFollowPathRequest,
    MessageUAVReplenishmentRequest,
    MessageWildlifeEncounterRequest,
)


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

        The following code pertains to the **2024 edition** of the AUVSI RobotX
        competition, held in Sarasota. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID identifying the message as a MIL heartbeat message.
            For the 2022 season, this is ``RXHRB``.
    """

    def __init__(self):
        self.message_id = "RXHRB"
        self.timestamp_last = None

    def from_string(self, delim: bytes, string: str) -> tuple[list[str], list[str]]:
        """
        From a message representing a message as a string, return the data and checksum
        lists encoded in the string.

        Args:
            delim (str): The delimiter separating the values in the data list. Frequently
                is a comma.
            string (str): The message represented as a string.

        Returns:
            Tuple[List[str], List[str]]: A tuple, where the first element is a list
            of data values, and the second element is the checksum info.
        """
        data_list = string.split(delim)
        checksum_list = string.split(b"*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: str,
        edt_date_time: Any,
        gps_array: Any,
        odom: Odometry | None,
        uav_status: int | None,
        system_mode: int | None,
        use_test_data: bool,
    ) -> str:
        """
        Given the necessary information to encode in the message, a message (as a string)
        is created. This message is ready to be sent back through the RobotX communications
        protocol.

        Args:
            delim (str): The delimiter to use when separating the data.
            team_id (str): The team ID used by MIL when sending messages.
            edt_date_time (Any): Presumably (??) a datetime object representing the
                current time in AEDT.
            gps_array (Optional[Any]): A specific message type containing at least a point. (??)
            odom (Optional[Odometry]): An optional odometry message to encode in the message.
                If ``None``, then empty strings are used in the message instead of the
                current position.
            uav_status (Optional[int]): The status of the UAV. If ``None``,
                then zero is used in the message.
            system_mode (Optional[int]): The current mode of the boat. If ``None``,
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

        if uav_status is None:
            uav_status = 0

        if system_mode is None:
            system_mode = 0

        first_half_data = f"{self.message_id}{delim}{edt_date_time}{delim}{latitude}{delim}{north_south}"

        second_half_data = f"{longitude}{delim}{east_west}{delim}{team_id}{delim}{system_mode}{delim}{uav_status!s}"

        full_data = first_half_data + delim + second_half_data

        # test data
        if use_test_data:
            full_data = "RXHRB,111221,161229,21.31198,N,157.88972,W,ROBOT,2,1"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(full_data)
        hex_checksum = format(checksum, "02X")

        msg_return = f"${full_data}*{str(hex_checksum).zfill(2)}\r\n"

        return msg_return


class RobotXEntranceExitGateMessage:
    """
    Handles formation of entrance and exit gates message.

    .. warning::

        The following code pertains to the **2024 edition** of the AUVSI RobotX
        competition, held in Sarasota. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message to signal that it is related to the
            entrance and exit gates.
    """

    def __init__(self):
        self.message_id = "RXGAT"

    def from_string(self, delim: bytes, string: str) -> tuple[list[str], list[str]]:
        """
        Constructs a list of data values and a checksum list from a provided message.

        Args:
            delim (str): The delimiter splitting up the data values.
            string (str): The message to get the data values from.

        Returns:
            Tuple[List[str], List[str]]: A tuple representing two values. The first
            is the list of data values encoded in the message. The second value
            is the checksum list encoded in the message.
        """
        data_list = string.split(delim)
        checksum_list = string.split(b"*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        edt_date_time: Any,
        data: MessageEntranceExitGateRequest,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a message using the provided parameters. This message is formatted
        according to 2022 AUVSI specifications.

        Args:
            delim (str): The delimiter to use in between data values.
            team_id (Any): A value (??) that can be converted to a string to represent
                the MIL team ID.
            edt_date_time (Any): A value (??) used to represent the current date + time
                in AEDT.
            data (MessageEntranceExitGateRequest): The data about the entrance/exit
                gate mission.
            use_test_data (bool): Whether to use test data in the message. If ``True``,
                then most of the other parameters are ignored.

        Returns:
            str: The encoded message.
        """

        data = f"{self.message_id}{delim}{edt_date_time}{delim}{team_id}{delim}{data.entrance_gate!s}{delim}{data.exit_gate!s}"

        # test data
        if use_test_data:
            data = "RXGAT,111221,161229,ROBOT,1,2"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = f"${data}*{hex_checksum}\r\n"

        return msg_return


class RobotXFollowPathMessage:
    """
    Handles formation of follow path message.

    .. warning::

        The following code pertains to the **2024 edition** of the AUVSI RobotX
        competition, held in Sarasota. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message to signal that it is related to the
            follow path message.
    """

    def __init__(self):
        self.message_id = "RXPTH"

    def from_string(self, delim: bytes, string: str) -> tuple[list[str], list[str]]:
        """
        Constructs a list of data values and a checksum list from a provided message.

        Args:
            delim (str): The delimiter splitting up the data values.
            string (str): The message to get the data values from.

        Returns:
            Tuple[List[str], List[str]]: A tuple representing two values. The first
            is the list of data values encoded in the message. The second value
            is the checksum list encoded in the message.
        """
        data_list = string.split(delim)
        checksum_list = string.split(b"*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        edt_date_time: Any,
        data: MessageFollowPathRequest,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a message using the provided parameters. This message is formatted
        according to 2022 AUVSI specifications.

        Args:
            delim (str): The delimiter to use in between data values.
            team_id (Any): A value (??) that can be converted to a string to represent
                the MIL team ID.
            edt_date_time (Any): A value (??) used to represent the current date + time
                in AEDT.
            data (MessageFollowPathRequest): The data about the follow path mission.
            use_test_data (bool): Whether to use test data in the message. If ``True``,
                then most of the other parameters are ignored.

        Returns:
            str: The encoded message.
        """

        data = f"{self.message_id}{delim}{edt_date_time}{delim}{team_id}{delim}{data.finished!s}"

        # test data
        if use_test_data:
            data = "RXPTH,111221,161229,ROBOT,1"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = f"${data}*{hex_checksum}\r\n"

        return msg_return


class RobotXWildlifeEncounterMessage:
    """
    Handles formation of the wildlife encounter message.

    .. warning::

        The following code pertains to the **2024 edition** of the AUVSI RobotX
        competition, held in Sarsota. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message to signal that it is related to the
            follow path message.
    """

    def __init__(self):
        self.message_id = "RXENC"

    def from_string(self, delim: bytes, string: str) -> tuple[list[str], list[str]]:
        """
        Constructs a list of data values and a checksum list from a provided message.

        Args:
            delim (str): The delimiter splitting up the data values.
            string (str): The message to get the data values from.

        Returns:
            Tuple[List[str], List[str]]: A tuple representing two values. The first
            is the list of data values encoded in the message. The second value
            is the checksum list encoded in the message.
        """
        data_list = string.split(delim)
        checksum_list = string.split(b"*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        edt_date_time: Any,
        data: MessageWildlifeEncounterRequest,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a message using the provided parameters. This message is formatted
        according to 2022 AUVSI specifications.

        Args:
            delim (str): The delimiter to use in between data values.
            team_id (Any): A value (??) that can be converted to a string to represent
                the MIL team ID.
            edt_date_time (Any): A value (??) used to represent the current date + time
                in AEDT.
            data (MessageWildlifeEncounterRequest): The data about the wildlife encounter mission.
            use_test_data (bool): Whether to use test data in the message. If ``True``,
                then most of the other parameters are ignored.

        Returns:
            str: The encoded message.
        """

        data_ = f"{self.message_id}{delim}{edt_date_time}{delim}{team_id}{delim}{len(data.buoy_array)!s}"

        for animal in data.buoy_array:
            data_ += delim + animal

        # test data
        if use_test_data:
            data_ = "RXENC,111221,161229,ROBOT,3,P,C,T"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data_)
        hex_checksum = format(checksum, "02X")

        msg_return = f"${data_}*{hex_checksum}\r\n"

        return msg_return


class RobotXScanCodeMessage:
    """
    Handles formation and sending of scan the code message.

    .. warning::

        The following code pertains to the **2024 edition** of the AUVSI RobotX
        competition, held in Sarasota. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID identifying the message as a Scan the Code message type.
            Equal to ``RXCOD``.
    """

    def __init__(self):
        self.message_id = "RXCOD"

    def from_string(self, delim: bytes, string: str) -> tuple[list[str], list[str]]:
        """
        Returns the information encoded in a message.

        Args:
            delim (str): The delimiter separating values in the data list encoded
                in the message.
            string (str): The message string to use to get data from.

        Returns:
            Tuple[List[str], List[str]]: The tuple containing a list of data values
            as the first value, and a list of checksums as the second value.
        """
        data_list = string.split(delim)
        checksum_list = string.split(b"*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        edt_date_time: Any,
        color_pattern: str,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a Scan the Code status message.

        Args:
            delim (str): The string delimiter used to separate distinct data
                points in the message.
            team_id (Any): The team ID used by MIL in the competition.
            edt_date_time (Any): The datetime to send in AEDT.
            color_pattern (str): The color pattern to send in the message.
            use_test_data (bool): Whether to use test data when sending the message.

        Returns:
            str: The constructed message.
        """
        data = f"{self.message_id}{delim}{edt_date_time}{delim}{team_id}{delim}{color_pattern}"

        # test data
        if use_test_data:
            data = "RXCOD,111221,161229,ROBOT,RBG"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = f"${data}*{hex_checksum}\r\n"

        return msg_return


class RobotXDetectDockMessage:
    """
    Handles formation of detect dock message.

    .. warning::

        The following code pertains to the **2024 edition** of the AUVSI RobotX
        competition, held in Sarasota. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message to signal that it is related to the
            detect dock message.
    """

    def __init__(self):
        self.message_id = "RXDOK"

    def from_string(self, delim: bytes, string: str) -> tuple[list[str], list[str]]:
        """
        Constructs a list of data values and a checksum list from a provided message.

        Args:
            delim (str): The delimiter splitting up the data values.
            string (str): The message to get the data values from.

        Returns:
            Tuple[List[str], List[str]]: A tuple representing two values. The first
            is the list of data values encoded in the message. The second value
            is the checksum list encoded in the message.
        """
        data_list = string.split(delim)
        checksum_list = string.split(b"*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        edt_date_time: Any,
        data: MessageDetectDockRequest,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a message using the provided parameters. This message is formatted
        according to 2022 AUVSI specifications.

        Args:
            delim (str): The delimiter to use in between data values.
            team_id (Any): A value (??) that can be converted to a string to represent
                the MIL team ID.
            edt_date_time (Any): A value (??) used to represent the current date + time
                in AEDT.
            data (MessageDetectDockRequest): The data about the detect dock mission.
            use_test_data (bool): Whether to use test data in the message. If ``True``,
                then most of the other parameters are ignored.

        Returns:
            str: The encoded message.
        """

        data = f"{self.message_id}{delim}{edt_date_time}{delim}{team_id}{delim}{data.color!s}{delim}{data.ams_status!s}"

        # test data
        if use_test_data:
            data = "RXDOK,111221,161229,ROBOT,R,1"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = f"${data}*{hex_checksum}\r\n"

        return msg_return


class RobotXFindFlingMessage:
    """
    Handles formation of find fling message.

    .. warning::

        The following code pertains to the **2024 edition** of the AUVSI RobotX
        competition, held in Sarasota. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message to signal that it is related to the
            find fling message.
    """

    def __init__(self):
        self.message_id = "RXFLG"

    def from_string(self, delim: bytes, string: str) -> tuple[list[str], list[str]]:
        """
        Constructs a list of data values and a checksum list from a provided message.

        Args:
            delim (str): The delimiter splitting up the data values.
            string (str): The message to get the data values from.

        Returns:
            Tuple[List[str], List[str]]: A tuple representing two values. The first
            is the list of data values encoded in the message. The second value
            is the checksum list encoded in the message.
        """
        data_list = string.split(delim)
        checksum_list = string.split(b"*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        edt_date_time: Any,
        data: MessageFindFlingRequest,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a message using the provided parameters. This message is formatted
        according to 2022 AUVSI specifications.

        Args:
            delim (str): The delimiter to use in between data values.
            team_id (Any): A value (??) that can be converted to a string to represent
                the MIL team ID.
            edt_date_time (Any): A value (??) used to represent the current date + time
                in AEDT.
            data (MessageFindFlingRequest): The data about the find fling mission.
            use_test_data (bool): Whether to use test data in the message. If ``True``,
                then most of the other parameters are ignored.

        Returns:
            str: The encoded message.
        """

        data = f"{self.message_id}{delim}{edt_date_time}{delim}{team_id}{delim}{data.color!s}{delim}{data.ams_status!s}"

        # test data
        if use_test_data:
            data = "RXFLG,111221,161229,ROBOT,R,2"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = f"${data}*{hex_checksum}\r\n"

        return msg_return


class RobotXUAVReplenishmentMessage:
    """
    Handles formation of UAV replenishment message.

    .. warning::

        The following code pertains to the **2024 edition** of the AUVSI RobotX
        competition, held in Sarasota. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message to signal that it is related to the
            uav replenishment message.
    """

    def __init__(self):
        self.message_id = "RXUAV"

    def from_string(self, delim: bytes, string: str) -> tuple[list[str], list[str]]:
        """
        Constructs a list of data values and a checksum list from a provided message.

        Args:
            delim (str): The delimiter splitting up the data values.
            string (str): The message to get the data values from.

        Returns:
            Tuple[List[str], List[str]]: A tuple representing two values. The first
            is the list of data values encoded in the message. The second value
            is the checksum list encoded in the message.
        """
        data_list = string.split(delim)
        checksum_list = string.split(b"*")
        return data_list, checksum_list

    def to_string(
        self,
        delim: str,
        team_id: Any,
        edt_date_time: Any,
        data: MessageUAVReplenishmentRequest,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a message using the provided parameters. This message is formatted
        according to 2022 AUVSI specifications.

        Args:
            delim (str): The delimiter to use in between data values.
            team_id (Any): A value (??) that can be converted to a string to represent
                the MIL team ID.
            edt_date_time (Any): A value (??) used to represent the current date + time
                in AEDT.
            data (MessageUAVReplenishmentRequest): The data about the WAV replenishment mission.
            use_test_data (bool): Whether to use test data in the message. If ``True``,
                then most of the other parameters are ignored.

        Returns:
            str: The encoded message.
        """

        data = f"{self.message_id}{delim}{edt_date_time}{delim}{team_id}{delim}{data.uav_status!s}{delim}{data.item_status!s}"

        # test data
        if use_test_data:
            data = "RXUAV,111221,161229,ROBOT,2,1"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = f"${data}*{hex_checksum}\r\n"

        return msg_return


class RobotXUAVSearchReportMessage:
    """
    Handles formation of UAV search report message.

    .. warning::

        The following code pertains to the **2024 edition** of the AUVSI RobotX
        competition, held in Sarasota. Updates to the specifications may have changed
        since this competition, and therefore, the code may not accurately represent
        the specifications MIL must produce for the competition.

    Attributes:
        message_id (str): The ID of the message to signal that it is related to the
            uav search report message.
    """

    def __init__(self):
        self.message_id = "RXSAR"

    def from_string(self, delim: bytes, string: str) -> tuple[list[str], list[str]]:
        """
        Constructs a list of data values and a checksum list from a provided message.

        Args:
            delim (str): The delimiter splitting up the data values.
            string (str): The message to get the data values from.

        Returns:
            Tuple[List[str], List[str]]: A tuple representing two values. The first
            is the list of data values encoded in the message. The second value
            is the checksum list encoded in the message.
        """
        data_list = string.split(delim)
        checksum_list = string.split(b"*")
        return data_list, checksum_list

    def lat_lon_ns(self, point: GeoPoint) -> tuple[float, str, float, str]:
        return (
            abs(point.latitude),
            "N" if point.latitude >= 0 else "S",
            abs(point.longitude),
            "E" if point.longitude >= 0 else "W",
        )

    def to_string(
        self,
        delim: str,
        team_id: Any,
        edt_date_time: Any,
        data: MessageUAVReplenishmentRequest,
        use_test_data: bool,
    ) -> str:
        """
        Constructs a message using the provided parameters. This message is formatted
        according to 2022 AUVSI specifications.

        Args:
            delim (str): The delimiter to use in between data values.
            team_id (Any): A value (??) that can be converted to a string to represent
                the MIL team ID.
            edt_date_time (Any): A value (??) used to represent the current date + time
                in AEDT.
            data (MessageUAVReplenishmentRequest): The data about the WAV replenishment mission.
            use_test_data (bool): Whether to use test data in the message. If ``True``,
                then most of the other parameters are ignored.

        Returns:
            str: The encoded message.
        """

        o1_lat, o1_ns, o1_lon, o1_ew = self.lat_lon_ns(data.object1_location)
        o2_lat, o2_ns, o2_lon, o2_ew = self.lat_lon_ns(data.object2_location)

        data = f"{self.message_id}{delim}{edt_date_time}{delim}{data.object1!s}{delim}{o1_lat!s}{delim}{o1_ns!s}{delim}{o1_lon!s}{delim}{o1_ew!s}{delim}{data.object2!s}{delim}{o2_lat!s}{delim}{o2_ns!s}{delim}{o2_lon!s}{delim}{o2_ew!s}{delim}{team_id}{delim}{data.uav_status!s}"

        # test data
        if use_test_data:
            data = "RXSAR,111221,161229,R,21.31198,N,157.88972,W,N, 21.32198,N,157.89972,W,ROBOT,2"

        checksum_calc = BitwiseXORChecksum()
        checksum = checksum_calc.ret_checksum(data)
        hex_checksum = format(checksum, "02X")

        msg_return = f"${data}*{hex_checksum}\r\n"

        return msg_return
