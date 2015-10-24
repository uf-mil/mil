#!/usr/bin/python
'''

This source is written for use in the Machine Intelligence Lab in the MAE
Department at the University of Florida. 
It is writen for use on the UF RobotX robot
It is released under the BSD license and is intended for university use
This code is provided "as is" and relies on specific hardware, use at your own risk

Title: Thrust Visualizer
Start Date: 2014

Author: All hail the mighty Jacob Panikulum
Author email: jpanikul@gmail.com

Edited By: Zach Goins
Date: 10-15-15
Email: zach.a.goins@gmail.com

CODE DETAILS --------------------------------------------------------------------

Please include inputs, outputs, and fill with a pseudo-code or description of the source to follow

inputs: /topic
output: /topic

1. Step 1
2. Step 2
3. Step 3
N. Step N

'''

## Math
from __future__ import division
import numpy as np
## Display
import pygame
import time
## Ros
import rospy
from tf import transformations as tf_trans
## AZI
import roslib; roslib.load_manifest('thrust_mapper')
from thrust_mapper import *
## Ros Msgs
from std_msgs.msg import Header, Float64, Float32MultiArray
from geometry_msgs.msg import Point, PointStamped, Quaternion
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3
from nav_msgs.msg import Odometry
from motor_control.msg import thrusterNewtons
from dynamixel_servo.msg import DynamixelFullConfig, DynamixelStatus

SCREEN_DIM = (750, 750)
ORIGIN = np.array([SCREEN_DIM[0] / 2.0, SCREEN_DIM[1] / 2.0])


def print_in(f):
    print("Defining " + f.func_name)
    def print_on_entry(*args, **kwargs):
        print("Executing " + f.func_name + " with arguments " + str(args) + " and kwargs " + str(kwargs))
        result = f(*args, **kwargs)
        print("Returning " + str(result))
        return(result)
    return(print_on_entry)


def round_point((x, y)):
    '''Round and change point to centered coordinate system'''
    return map(int, ((30 * y) + ORIGIN[0], -(30 * x) + ORIGIN[1]))


def unround_point((x, y)):
    '''Change center-origin coordinates to pygame coordinates'''
    return (-(y - ORIGIN[0]) / 30.0, -(-x + ORIGIN[1]) / 30.0)


def norm_angle_diff(ang_1, ang_2):
    '''norm_angle_diff(ang_1, ang_2)
    -> Normalized angle difference, constrained to [-pi, pi]'''
    return (ang_1 - ang_2 + np.pi) % (2 * np.pi) - np.pi


def saneify_angle(boat_angle):
    '''Make boat-style servo angles into something more...sane'''
    real_angle = (boat_angle - np.pi) / 2
    return real_angle


class Azi_Drive_Visual(object):
    def __init__(self):
        '''This intends to simulate the physical servo angles without fake offsets'''
        rospy.init_node('azi_drive_visualization')

        rospy.Subscriber('thruster_config', thrusterNewtons, 
            self._thruster_cb, queue_size=4)
        rospy.Subscriber('dynamixel/dynamixel_full_config', DynamixelFullConfig, 
            self._dynamixel_cb, queue_size=4)
        rospy.Subscriber('wrench', WrenchStamped, self._wrench_cb, queue_size=1)
        rospy.Subscriber('dynamixel/dynamixel_status_post', DynamixelStatus, self._dynamixel_cur_cb, queue_size=4)

        rospy.Subscriber('odom', Odometry, self._odometry_cb, queue_size=4)

        rospy.Subscriber('/motors', Float32MultiArray, self.mot_cb)

        self.thrusts = (0,0)

        # Thruster 2 is on the right, thruster 3 is on the left
        self.thruster_forces = {2: 0, 3: 0}
        self.thruster_goal_angles = {2: 0.0, 3: 0.0}
        self.thruster_cur_angles = {2: 45, 3:0.0}
        self.thruster_angles = {2: np.pi/2, 3: np.pi/2}
        self.wrench = 0

        self.pos_x, self.pos_y = 0.0, 0.0
        self.yaw = 0.0


        self.thruster_positions = {
            # Taken from simulation
            3: (-0.7239, -0.3048),
            2: (-0.7239, 0.3048),
        }

    def mot_cb(self, msg):
        self.thruster_forces[2] = msg.data[0]
        self.thruster_forces[3] = msg.data[1]

    def _odometry_cb(self, msg):
        quat2quat = lambda quat: (quat.x, quat.y, quat.z, quat.w)
        self.pos_x, self.pos_y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.yaw = tf_trans.euler_from_quaternion(quat2quat(msg.pose.pose.orientation))[2]

    def _thruster_cb(self, msg):
        self.thruster_forces[msg.id] = msg.thrust

    def _wrench_cb(self, msg):
        pass

    def _dynamixel_cb(self, msg):
        self.thruster_goal_angles[msg.id] = msg.goal_position

    def _dynamixel_cur_cb(self, msg):
        if msg.id in self.thruster_cur_angles.keys():
            self.thruster_cur_angles[msg.id] = msg.present_position

    def ray(self, display, start, angle, length, scale=0.1, color=(255, 0, 0), text=None, width=3):
        unit_v = np.array([np.sin(angle), np.cos(angle)])
        _start = round_point(start)
        _end = round_point(start + (unit_v * length * scale))


        pygame.draw.circle(display, (255, 200, 80), _start, 5)

        pygame.draw.line(display, color,
            _start,
            _end,
            width
        )

        if text is not None:
            Text_Box.draw(display, 
                pos=_end,
                color=(255, 0, 0), 
                text=text
            )

    def vector(self, display, start, direction, scale=0.1):
        pygame.draw.circle(display, (200, 255, 20), round_point(start), 5)
        pygame.draw.line(display, (0, 255, 0),
            round_point(start),
            round_point(np.array(start) + (np.array(direction) * scale)),
        )

    def draw(self, display):
        # Update positions given current angles
        for _id, position in self.thruster_positions.items():
            self.ray(display, 
                start=position, 
                angle=saneify_angle(self.thruster_cur_angles[_id]) - (np.pi / 2),
                length=10,
                scale=0.25,
                color=(50, 30, 255),
                width=5
            )        

            angle = self.thruster_angles[_id]
            length = self.thruster_forces[_id]
            self.ray(display, 
                start=position, 
                angle=angle, 
                length=-length,
                scale=0.1,
                text="{}Force: {}\nAngle: {}\n".format(
                    "\n\n" * (_id - 2),
                    round(length, 4),
                    round(angle, 4),
                ),
            )        



class Text_Box(object):
    '''Text_Box()
    You never have to initialize this! Just call Text_Box.draw(display, pos, color, text)
    It draws the same way a pygame primitive would.
    '''
    pygame.font.init()
    font = pygame.font.SysFont("monospace", 15)


    @classmethod
    def draw(self, display, pos=(0, 0), color=(255, 255, 255), text="Empty!"):
        ''' draw(display, pos=(0, 0), color=(255, 255, 255), text="Empty!"):

        pos: In pygame coordinates
        color: [0, 255]
        text: Can by multiline, of arbitrary length

        To change text during operation, use the "set_text method"
        Ex:
            >>> tb = Text_Box()
            >>> tb.draw(display, text='hello')
        or in a draw loop,
            >>> tb.draw(display, pos, color, text)
        '''
        lines = text.splitlines()
        width = height = 0
        for l in lines:
            width = max(width, self.font.size(l)[0])
            height += self.font.get_linesize()

        height = 0
        for l in lines:
            t = self.font.render(l, 0, color)
            display.blit(
                t, 
                (pos[0], pos[1] + height)
            )
            height += self.font.get_linesize()


wrench_pub = rospy.Publisher('wrench', WrenchStamped, queue_size=2)

def publish_wrench(fx, fy, tau):
    wrench = WrenchStamped()
    wrench.wrench = Wrench()
    wrench.wrench.force = Vector3()
    wrench.wrench.torque = Vector3()

    wrench.wrench.force.x = fx
    wrench.wrench.force.y = fy
    wrench.wrench.force.z = 0
    
    wrench.wrench.torque.x = 0
    wrench.wrench.torque.y = 0
    wrench.wrench.torque.z = tau

    wrench.header.seq = 0
    wrench.header.frame_id = '/base_link'
    wrench.header.stamp = rospy.Time.now()

    wrench_pub.publish(wrench)


def main():
    draws = [
        Azi_Drive_Visual()
    ]

    display = pygame.display.set_mode(SCREEN_DIM)
    pygame.display.set_caption("Boat azi_drive_visualization")

    clock = pygame.time.Clock()

    last_time = 0
    targeted = False
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

            if event.type == pygame.KEYDOWN:
                if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                    return

            if event.type == pygame.MOUSEBUTTONDOWN:
                if not pygame.mouse.get_pressed()[0]:
                    continue

                last_time = time.time()
                pos = pygame.mouse.get_pos()
                point = unround_point(pos)
                publish_wrench(
                    fx=point[0] * 5, 
                    fy=point[1] * 5, 
                    tau=0.0,
                )
                targeted = True

            if pygame.mouse.get_pressed()[0]:
                last_time = time.time()

        t = time.time()
        if (t - last_time > 2) and targeted:
            rospy.logwarn("Stopping")
            publish_wrench(0, 0, 0)
            targeted = False

        for draw in draws:
            draw.draw(display)
        
        pygame.display.update()
        clock.tick(20)
        display.fill((0, 0, 0))


if __name__ == '__main__':
    main()