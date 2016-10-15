import rospy

def print_t(_str, time=None):
    # Needs a rospy init for ros time
    if time is None:
        time = rospy.Time.now().to_sec()

    print "\033[1m{}:\033[0m {}".format(time, _str)