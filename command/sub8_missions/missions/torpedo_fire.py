from txros import util
import rospy
from geometry_msgs.msg import Point
from mil_ros_tools import rosmsg_to_numpy
from mil_msgs.msg import RangeStamped

# Current depth constant
CURRENT_DEPTH = 0


@util.cancellableInlineCallbacks
def check_depth():
    if (CURRENT_DEPTH - .15 < 1):
        return False
    else:
        return True


@util.cancellableInlineCallbacks
def update_point_callback(msg, sub):
    m = msg

    if m.z == -6:
        print "up left"
        yield sub.move.up(.15).go()
        yield sub.move.left(.15).go()

    elif m.z == -5:
        print "up"
        yield sub.move.up(.15).go()

    elif m.z == -4:
        print "up right"
        yield sub.move.up(.15).go()
        yield sub.move.right(.15).go()

    elif m.z == -1:
        print "left"
        yield sub.move.left(.15).go()

    elif m.z == 1:
        print "right"
        yield sub.move.right(.15).go()

    elif m.z == 4:
        print "down left"
        if (check_depth()):
            yield sub.move.down(.15).go()
        yield sub.move.left(.15).go()

    elif m.z == 5:
        print "down"
        if (check_depth()):
            yield sub.move.down(.15).go()

    elif m.z == 6:
        print "down right"
        if (check_depth()):
            yield sub.move.down(.15).go()
        yield sub.move.right(.15).go()

    elif m.z == 0:
        print "Target Lock Aquired. Firing Ze Missiles."
        # continue
        # fire the missiles!
        # We good bois.
    else:
        print("Danger Will Robinson")
        # continue
        # welp.
        # return True


@util.cancellableInlineCallbacks
def depth_callback(msg):
    CURRENT_DEPTH = msg.range


@util.cancellableInlineCallbacks
def run(sub):

    # dive to mission start depth
    # mission_start_depth = float(input(
    #     "Entered the desired depth for the 'set course' mission: "))  # For pool testing
    # # mission_start_depth = 2.15        # meters
    print "descending to set course mission depth"
    yield sub.move.to_height(1).go()

    rospy.init_node('torpedo_mission', anonymous=True)

    point_sub = rospy.Subscriber(
        "torp_vision/points", Point, update_point_callback, callback_args=sub)

    current_depth = rospy.Subscriber("mil_msgs/RangeStamped", RangeStamped,
                                     depth_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    '''
    Possible Z values and their meanings:
    m.z = 0 --> Victory.

    ### X ### Range of Values: (-5, 5)
    m.z = -5 --> Only X thresh is off and sub is too low.
    m.z = 5 --> Only X thresh is off and sub is too high.

    ### BOTH ### Range of Values: (4, 6, -4, -6)
    m.z = 4 --> Both are off, too high, too far right.
    m.z = 6 --> Both are off, too high, too far left.
    m.z = -4 --> Both are off, too low, too far left.
    m.z = -6 --> Both are off, too low, too far right. 

    ### Y ### Range of Values(-1, 1)
    m.z = -1 --> Only Y is off, too far right.
    m.z = 1 --> Only Y is off, too far left.
    '''
