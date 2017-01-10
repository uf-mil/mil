import rospy
from ros_alarms import HandlerBase

class Tester(HandlerBase):
    alarm_name = "testing"

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
