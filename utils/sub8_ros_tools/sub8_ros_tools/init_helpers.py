import rospy
from time import time


def wait_for_param(param_name, timeout=None, poll_rate=0.1):
    '''Blocking wait for a parameter named $parameter_name to exist
        Poll at frequency $poll_rate
        Once the parameter exists, return get and return it.

    This function intentionally leaves failure logging duties to the developer
    '''
    start_time = time()
    rate = Rospy.Rate(poll_rate)
    while not rospy.is_shutdown():

        # Check if the parameter now exists
        if rospy.has_param(param_name):
            return rospy.get_param(param_name)

        # If we exceed a defined timeout, return None
        if timeout is not None:
            if time() - start_time > timeout:
                return None

        # Continue to poll at poll_rate
        rate.sleep()