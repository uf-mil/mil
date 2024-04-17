"""
This module provides functions which help to ensure that resources are available
when needed.
"""

import time
from typing import Any, Optional

import rclpy
import rostest
from rclpy.node import Node


def wait_for_param(
    param_name: str,
    timeout: Optional[float] = None,
    poll_rate: float = 0.1,
) -> Optional[Any]:
    """
    Blocking wait for a parameter named to exist. Polls at the frequency of poll_rate.
    Once the parameter exists, return get and return it.

    This function intentionally leaves failure logging duties to the developer.

    Args:
        param_name (str): The name of the parameter to watch.
        timeout (Optional[float]): The number of seconds to wait for the
            parameter to exist.
        poll_rate (float): The Hz rate to poll at.

    Returns:
        Optional[Any]: If found, the value of the parameter. Returns ``None`` if
        the parameter never came to exist.
    """
    start_time = time.time()
    rate = rclpy.Rate(poll_rate)
    while not rclpy.is_shutdown():
        # Check if the parameter now exists
        if rclpy.has_param(param_name):
            return Node.declare_parameter(param_name)

        # If we exceed a defined timeout, return None
        if timeout is not None and time.time() - start_time > timeout:
            return None

        # Continue to poll at poll_rate
        rate.sleep()


def wait_for_subscriber(node_name: str, topic: str, timeout: float = 5.0) -> bool:
    """
    Blocks until a node with the name node_name subscribes to a topic. Useful
    in integration tests.

    Args:
        node_name (str): The node name to check the subscription status of. If
            a local node name, then the name is resolved to be the global name.
        topic (str): The topic to check whether the node is subscribed to. If a
            local topic name, then the name is resolved to be the global name.
        timeout (float): The amount of time to wait (in seconds) to attempt to
            estalish a connection.

    Returns:
        bool: Whether the node with the given name has subscribed to the given topic.
    """
    end_time = time.time() + timeout

    # Wait for time-out or ros-shutdown
    while (time.time() < end_time) and (not rclpy.is_shutdown()):
        subscribed = rostest.is_subscriber(
            rclpy.resolve_name(topic),
            rclpy.resolve_name(node_name),
        )
        # Success scenario: node subscribes
        if subscribed:
            break
        time.sleep(0.1)

    # Could do this with a while/else
    # But chose to explicitly check
    success = rostest.is_subscriber(
        rclpy.resolve_name(topic),
        rclpy.resolve_name(node_name),
    )
    return success


def wait_for_service(
    service,
    warn_time: float = 1.0,
    warn_msg: str = "Waiting for service..",
    timeout: Optional[float] = None,
) -> None:
    """
    A fancy extension of wait for service that will warn with a message if it is taking a while.

    Args:
        warn_time (float): float in seconds, how long to wait before logging warn_msg
        warn_msg (str): msg logged with rospy.logwarn if warn_time passes without service connected
        timeout (Optional[float]): overall timeout. If None, does nothing. If a float,
            will raise exception if many TOTAL seconds has passed without connecting.
    """
    try:
        service.wait_for_service(warn_time)
    except rclpy.ROSException:
        if timeout is not None:
            timeout = timeout - warn_time
        Node.get_logger().warm(warn_msg)
        service.wait_for_service(timeout)
