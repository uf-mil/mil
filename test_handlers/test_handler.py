import rospy
from ros_alarms import HandlerBase

class Tester(HandlerBase):
    alarm_name = "testing"

    def __init__(self):
        rospy.Timer(rospy.Duration(1), self.do_stuff)

    def do_stuff(self, *args):
        print "Alive"

    def raised(self, alarm):
        print "Tester raised"

    def cleared(self, alarm):
        print "Tester cleared"

class Tester2(HandlerBase):
    alarm_name = "testing1"

    def raised(self, alarm):
        print "Tester2 raised"

    def cleared(self, alarm):
        print "Tester2 cleared"

class Tester3(object):
    pass
