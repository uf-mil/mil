#!/usr/bin/python

from mil_misc_tools import text_effects
import rospy
import argparse
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped
from std_msgs.msg import Header


def zero():
    return Vector3(0, 0, 0)

if __name__ == '__main__':
    desc_msg = "Tool to quickly format and publish a wrench"
    parser = argparse.ArgumentParser(description=desc_msg)
    parser.add_argument(
        dest='direction', metavar='direction',
        help='Single direction for thrust or torque: x,y,z,rol,pit,yaw'
    )
    parser.add_argument(
        dest='scale', type=float, metavar='scale',
        help='float value for scaling force/torque'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    direction = args.direction
    scale = args.scale
    rospy.init_node('quick_wrench')
    pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=1)
    options = {'x': Wrench(force=Vector3(scale, 0, 0), torque=zero()),
               'y': Wrench(force=Vector3(0, scale, 0), torque=zero()),
               'z': Wrench(force=Vector3(0, 0, scale), torque=zero()),
               'rol': Wrench(force=zero(), torque=Vector3(scale, 0, 0)),
               'pit': Wrench(force=zero(), torque=Vector3(0, scale, 0)),
               'yaw': Wrench(force=zero(), torque=Vector3(0, 0, scale))}
    interesting = rospy.Rate(20)
    while not rospy.is_shutdown():
        wrench = WrenchStamped(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='/base_link'),
            wrench=options[direction])
        pub.publish(wrench)
        interesting.sleep()
