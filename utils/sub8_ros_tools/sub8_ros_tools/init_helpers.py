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
