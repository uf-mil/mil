#!/usr/bin/env python
import sys
import rospy
from sub8_msgs.srv import GuessRequest, GuessRequestResponse
from geometry_msgs.msg import PoseStamped


class Guess:

    def __init__(self):
        rospy.sleep(1.0)
        self.items = ['pinger1', 'pinger2']
        self.locations = []
        self.guess_service = rospy.Service('guess_location', GuessRequest, self.request_location)
        self.clicked_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.add_guess_to_list)

    def request_location(self, srv):
        req_item = srv.item
        if (req_item in self.items):
            return GuessRequestResponse(location=self.locations[self.items.index(req_item)], found=True)
        else:
            return GuessRequestResponse(found=False)

    def add_guess_to_list(self, msg):
        print('Added Item')
        self.locations.append(msg)


def main(args):
    Guess()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('guess')
    main(sys.argv)
