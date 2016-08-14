#!/usr/bin/python

import time
import traceback
import socket
import json
import numpy

import rospy

from std_msgs.msg import Header, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance, Point, Vector3, Quaternion, TransformStamped, Transform
from tf.msg import tfMessage


rospy.init_node('sdgps_solution_ros_bridge')

host = rospy.get_param('~host', '127.0.0.1')
port = rospy.get_param('~port')
child_frame_id = rospy.get_param('~child_frame_id')
decimation = rospy.get_param('~decimation', 1)

odom_pub = rospy.Publisher('odom', Odometry)
absodom_pub = rospy.Publisher('absodom', Odometry)
clock_error_pub = rospy.Publisher('clock_error', Float64)

tf_pub = rospy.Publisher('/tf', tfMessage)

def go():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    
    first = True
    
    buf = ''
    count = 0
    while True:
        d = s.recv(2**12)
        if not d:
            break
        buf += d
        
        lines = buf.split('\n')
        buf = lines[-1]
        for line in lines[:-1]:
            if first:
                first = False
                continue
            
            if count % decimation == 0:
                d = json.loads(line)
                
                ecef_cov = numpy.array(d['X_position_relative_position_orientation_ecef_covariance'])
                absodom_pub.publish(Odometry(
                    header=Header(
                        stamp=rospy.Time.from_sec(d['timestamp']*1e-9),
                        frame_id='/ecef',
                    ),
                    child_frame_id=child_frame_id,
                    pose=PoseWithCovariance(
                        pose=Pose(
                            position=Point(*d['position_ecef']),
                            orientation=Quaternion(**d['orientation_ecef']),
                        ),
                        covariance=numpy.vstack((
                            numpy.hstack((ecef_cov[0:3, 0:3], ecef_cov[0:3, 6:9])),
                            numpy.hstack((ecef_cov[6:9, 0:3], ecef_cov[6:9, 6:9])),
                        )).flatten(),
                    ),
                    twist=TwistWithCovariance(
                        twist=Twist(
                            linear=Vector3(*d['velocity_body']),
                            angular=Vector3(*d['angular_velocity_body']),
                        ),
                        covariance=numpy.vstack((
                            numpy.hstack((d['X_velocity_body_covariance'], numpy.zeros((3, 3)))),
                            numpy.hstack((numpy.zeros((3, 3)), d['X_angular_velocity_body_covariance'])),
                        )).flatten(),
                    ),
                ))
                odom_pub.publish(Odometry(
                    header=Header(
                        stamp=rospy.Time.from_sec(d['timestamp']*1e-9),
                        frame_id='/enu',
                    ),
                    child_frame_id=child_frame_id,
                    pose=PoseWithCovariance(
                        pose=Pose(
                            position=Point(*d['relative_position_enu']),
                            orientation=Quaternion(**d['orientation_enu']),
                        ),
                        covariance=numpy.array(d['X_relative_position_orientation_enu_covariance']).flatten(),
                    ),
                    twist=TwistWithCovariance(
                        twist=Twist(
                            linear=Vector3(*d['velocity_body']),
                            angular=Vector3(*d['angular_velocity_body']),
                        ),
                        covariance=numpy.vstack((
                            numpy.hstack((d['X_velocity_body_covariance'], numpy.zeros((3, 3)))),
                            numpy.hstack((numpy.zeros((3, 3)), d['X_angular_velocity_body_covariance'])),
                        )).flatten(),
                    ),
                ))
                clock_error_pub.publish(Float64(d['X_clock_error']))
                tf_pub.publish(tfMessage(
                    transforms=[
                        TransformStamped(
                            header=Header(
                                stamp=rospy.Time.from_sec(d['timestamp']*1e-9),
                                frame_id='/enu',
                            ),
                            child_frame_id=child_frame_id,
                            transform=Transform(
                                translation=Point(*d['relative_position_enu']),
                                rotation=Quaternion(**d['orientation_enu']),
                            ),
                        ),
                    ],
                ))
            
            count += 1

while True:
    try:
        go()
    except Exception:
        traceback.print_exc()
        time.sleep(1)
