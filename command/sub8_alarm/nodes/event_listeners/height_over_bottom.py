#!/usr/bin/env python
import rospy
from sub8_alarm import single_alarm
from uf_common.msg import Float64Stamped

class HeightOverBottom(object):
    '''Kills the sub if it get too low.
    Set the param /height_over_bottom_kill to the height in meters that the sub should kill at
    '''

    def __init__(self, height):
        self.height = height
        self.too_low = False

        self.alarm_broadcaster, self.alarm = single_alarm('height-over-bottom')
        rospy.Subscriber('/dvl/range', Float64Stamped, self.got_range)

    def got_range(self, msg):
        curr_height = msg.data

        # Raise alarm when we've gone too low
        if curr_height <= self.height and not self.too_low:
            self.too_low = True
            self.alarm.raise_alarm()
            rospy.logwarn("TOO LOW!")

        # Clear alarm when we've raised up a bit
        elif curr_height >= self.height + 0.5 and self.too_low:
            self.too_low = False
            self.alarm.clear_alarm()

if __name__ == "__main__":
    rospy.init_node("height_over_bottom_alarm_kill")
    HeightOverBottom(rospy.get_param("/height_over_bottom_kill_m", 0.4))
    rospy.spin()
