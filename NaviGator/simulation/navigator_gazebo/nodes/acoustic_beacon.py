import rospy
#from usv_msgs.msg import RangeBearing
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from navigator_msgs.srv import AcousticBeacon, AcousticBeaconResponse
import math

#def __init__(self, *args):
#    rospy.Subscriber("/odom", Odometry, self.odom)
#    rospy.Subscriber("/wamv/sensors/pingers/pinger/range_bearing", Ran


def acoustic_locator_server():
    rospy.init_node("acoustic_beacon")
    s = rospy.Service("acoustic_beacon", AcousticBeacon, handler)
    rospy.spin()

def handler(req):
    self.pos = req.position
    self.rot = req.orientation
    self.range = req.range
    self.bearing = req.bearing
    self.elevation = req.elevation

    #[x, y, z]
    self.gps = Point(0)
        
    f = self.range * cos(self.elevation)
    theta = 90 - self.bearing
    self.gps[0] = f * cos(theta)
    self.gps[1] = f * sin(theta)

    return AcousticBeaconResponse(gps)


if __name__ == '__main__':
    acoustic_locator_server()
