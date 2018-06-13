from txros import util
import numpy as np

TARGET_LOCK = False
LOCK_COUNTER = 0
LOCK_THRESH = 10

DIFF_THRESH = 15


@util.cancellableInlineCallbacks
def update_point_callback(msg, sub, l_x, l_y, l_z):
    global TARGET_LOCK
    global LOCK_COUNTER
    global LOCK_THRESH
    global DIFF_THRESH

    m = msg

    x = m.x
    y = m.y
    r = m.z

    if (abs(x - l_x) < DIFF_THRESH) and (abs(y - l_y) < DIFF_THRESH) and (abs(r - l_z) < DIFF_THRESH):
        # Its probably the same circle.
        LOCK_COUNTER += 1

        if LOCK_COUNTER == LOCK_THRESH:
            while(True):
                if abs(response.pose.x) > 10 or abs(response.pose.y) > 10:
                    if response.pose.x < 0:
                        yield sub.move.left(abs(response.pose.x / 500)).go()
                    if response.pose.x > 0:
                        yield sub.move.right(abs(response.pose.x / 500)).go()
                    if response.pose.y > 0:
                        yield sub.move.forward(abs(response.pose.y / 500)).go()
                    if response.pose.y < 0:
                        yield sub.move.backward(abs(response.pose.y / 500)).go()
                else:
                    break
        else:
            DIFF_THRESH = r

            return x, y, r


@util.cancellableInlineCallbacks
def run(sub_singleton):
    '''
    Cause the sub to perform a compass box search pattern. This is a premade search pattern in the Searcher class. 
    The sub moves forward i * .5 meters, turns left 90 degrees. It moves forward i * .5 meters again and then rotates left another 90 degrees. 
    The loop then ticks up by one, increasing a coefficient. i is instantiated with a value of 1. 
    When the loop ticks up i becomes 2, then 3, and so on. Thus the sub would move forward 1 meter, then 1.5 meters respectively. 
    Below is a graphic of how this would look.
    https://en.wikipedia.org/wiki/Underwater_searches#/media/File:Compass_box_search_pattern.png 
    '''
    print "Begin searching for the roulette wheel."
    # yield sub.move.to_height(1).go(blind=true)
    global TARGET_LOCK

    # rospy.init_node('torpedo_mission', anonymous=True)
    # Subscriber for Depth.

    range_sub = sub.nh.subscribe("/dvl/range", RangeStamped)

    # Subscriber for my interesting logic system/perception.

    points_sub = yield sub.nh.subscribe(
        "vision/roulette_wheel/points", Point)

    while TARGET_LOCK == False:
        print "Attempting to get Range"
        range_msg = range_sub.get_next_message()
        print "depth_callback"
        depth_callback(range_msg)
        print "Attempting to get Points"
        point_msg = yield points_sub.get_next_message()
        print "update_point_callback"
        l_x, l_y, l_z = update_point_callback(point_msg, sub, l_x, l_y, l_z)

    print "Done!"
