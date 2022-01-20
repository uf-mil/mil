#!/usr/bin/env python
#this is a cameron problem https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/
import rospy
from usv_msgs.msg import RangeBearing
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from navigator_msgs.srv import AcousticBeacon, AcousticBeaconResponse
import math
import tf

#Defines a service that returns the position of the acoustic beacon from odom and the range_bearing topic
class AcousticBeaconLocator():
    def __init__(self):
        rospy.init_node("acoustic_beacon")
        self.odom = rospy.Subscriber('/odom', Odometry, self.odometrySubscriber)
        self.range = rospy.Subscriber('/wamv/sensors/pingers/pinger/range_bearing', RangeBearing, self.rangeSubscriber)
        #self.pub = rospy.Publisher('/beaconLocator', Point, queue_size=10)
        self.serv = rospy.Service('beaconLocator', AcousticBeacon, self.handler)
        self.pos = Point()
        self.orientation = Quaternion()
        self.range = 0.0
        self.bearing = 0.0
        self.elevation = 0.0
        self.odomSet = False
        self.rangeSet = False
        rospy.spin()

    #Requires both odom and range_bearing to be publishing data
    def odometrySubscriber(self, msg):
        self.pos = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.odomSet = True

    def rangeSubscriber(self, msg):
        self.range = msg.range
        self.bearing = msg.bearing
        self.elevation = msg.elevation
        self.rangeSet = True

    #If odom and range_bearing haven't published yet then return default values of 0
    def handler(self, res):
        #[x, y, z]
        gps = AcousticBeaconResponse()
        if not(self.rangeSet and self.odomSet):
            gps.setValue.data = False
            return gps
        gps.setValue.data = True
        orientation_list = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        
        f = self.range * math.cos(self.elevation)
        theta = yaw - self.bearing
        gps.beacon_position.x = f * math.cos(theta)
        gps.beacon_position.y = f * math.sin(theta)

        return gps


if __name__ == '__main__':
    AcousticBeaconLocator()
