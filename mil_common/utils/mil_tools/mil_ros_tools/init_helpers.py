import rospy
import rostest
import time


def wait_for_param(param_name, timeout=None, poll_rate=0.1):
    '''Blocking wait for a parameter named $parameter_name to exist
        Poll at frequency $poll_rate
        Once the parameter exists, return get and return it.

    This function intentionally leaves failure logging duties to the developer
    '''
    start_time = time.time()
    rate = rospy.Rate(poll_rate)
    while not rospy.is_shutdown():

        # Check if the parameter now exists
        if rospy.has_param(param_name):
            return rospy.get_param(param_name)

        # If we exceed a defined timeout, return None
        if timeout is not None:
            if time.time() - start_time > timeout:
                return None

        # Continue to poll at poll_rate
        rate.sleep()


def wait_for_subscriber(node_name, topic, timeout=5.0):
    '''Blocks until $node_name subscribes to $topic
    Useful mostly in integration tests --
        I would counsel against use elsewhere
    '''
    end_time = time.time() + timeout

    # Wait for time-out or ros-shutdown
    while (time.time() < end_time) and (not rospy.is_shutdown()):
        subscribed = rostest.is_subscriber(
            rospy.resolve_name(topic),
            rospy.resolve_name(node_name)
        )
        # Success scenario: node subscribes
        if subscribed:
            break
        time.sleep(0.1)

    # Could do this with a while/else
    # But chose to explicitly check
    success = rostest.is_subscriber(
        rospy.resolve_name(topic),
        rospy.resolve_name(node_name)
    )
    return success


def wait_for_service(service, warn_time=1.0, warn_msg='Waiting for service..', timeout=None):
    '''
    A fancy extension of wait for service that will warn with a message if it is taking a while.

    @param warn_time: float in seconds, how long to wait before logging warn_msg
    @param warn_msg: msg logged with rospy.logwarn if warn_time passes without service connected
    @param timeout: overall timeout. If None, does nothing. If a float, will raise exception
                    if many TOTAL seconds has passed without connecting
    '''
    try:
        service.wait_for_service(warn_time)
    except rospy.ROSException:
        if timeout is not None:
            timeout = timeout - warn_time
        rospy.logwarn(warn_msg)
        service.wait_for_service(timeout)
