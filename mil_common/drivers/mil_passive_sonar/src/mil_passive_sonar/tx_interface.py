#!/usr/bin/env python3
from typing import Callable

from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest
from twisted.internet import defer
from txros import NodeHandle, util
from txros.serviceclient import ServiceClient


class TxHydrophonesClient:
    """
    TxROS abstraction for interacting with the nodes in this package.

    Attributes:
        dir_callback (Optional[Callable]): The method that is called when a ping
            is received.
    """

    def __init__(self, nh: NodeHandle):
        """
        Construct a client.

        Args:
            nh (NodeHandle): The TxROS node handler.
        """
        self._direction_sub = nh.subscribe(
            "/hydrophones/direction", Vector3Stamped, callback=self.heading_cb
        )
        self._enable_srv = nh.get_service_client("/multilateration/enable", SetBool)
        self._reset_srv = nh.get_service_client("/multilateration/reset", Trigger)
        self._position_sub = nh.subscribe("/hydrophones/position", PointStamped)
        self.dir_callback = None

    def get_direction(self) -> defer.Deferred:
        """
        Get the next processed direction to the pinger.

        Returns:
            defer.Deferred: A deferred object which can be used to get the next message
                of the pinger's direction.
        """
        return self._direction_sub.get_next_message()

    def get_position(self) -> defer.Deferred:
        """
        Get the next processed position of the pinger.

        Returns:
            defer.Deferred: A deferred object which can be used to get the next
                message of the pinger's position.
        """
        return self._position_sub.get_next_message()

    def get_last_position(self) -> PointStamped:
        """
        Get the last processed position of the pinger.

        Returns:
            PointStamped: A cached value of the last position of the pinger.
        """
        return self._position_sub.get_last_message()

    def enable(self) -> None:
        """
        Enable listening to pings for position estimation.
        """
        return self._enable_srv(SetBoolRequest(data=True))

    def reset(self) -> None:
        """
        Reset the position estimation of the pinger.
        """
        return self._reset_srv(TriggerRequest())

    def disable(self) -> None:
        """
        Disable listening to pings for position estimation.
        """
        return self._enable_srv(SetBoolRequest(data=False))

    def heading_cb(self, heading_msg: Vector3Stamped):
        """
        Callback for pings received.

        Args:
            heading_msg (Vector3Stamped): The passed-in message representing the
                heading.
        """
        if self.dir_callback is not None:
            self.dir_callback(heading_msg)

    def set_callback(self, cb: Callable):
        """
        Set a callback for when a ping is received.

        Args:
            cb (Callable): The callback to set. The callback should have a single
                parameter that takes a Vector3Stamped message representing the heading
                towards the pinger.
        """
        self.dir_callback = cb
