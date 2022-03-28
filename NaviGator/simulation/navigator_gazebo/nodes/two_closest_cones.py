#!/usr/bin/env python
#this is a cameron problem https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/
from numpy import Inf
import rospy
from nav_msgs.msg import Odometry
from navigator_msgs.srv import TwoClosestCones, TwoClosestConesResponse
from mil_msgs.msg import PerceptionObjectArray
from mil_tools import pose_to_numpy, rosmsg_to_numpy
import math
import numpy as np
import tf

#Defines a service that returns the position of the acoustic beacon from odom and the range_bearing topic
class PcodarAverageScales():
    def __init__(self):

        #create service server
        self.server = rospy.Service('/get_two_closest_cones', TwoClosestCones, self.handler)
        
        #create service client
        self.odom = rospy.Subscriber('/pcodar/objects', PerceptionObjectArray, self.pcodarSubscriber)

        #create odom subscriber
        self.odom = rospy.Subscriber('/odom', Odometry, self.odometrySubscriber)

        self.scale_x_avg = []
        self.scale_x_count = []
        self.scale_y_avg = []
        self.scale_y_count = []
        self.scale_z_avg = []
        self.scale_z_count = []
        self.pos = None
        self.ori = None
        self.odomSet = False
        self.scalesSet = False
        self.closest_two_dists = [None,None]
        self.closest_two_indicies = [None,None]

    def odometrySubscriber(self, msg):
        pose = msg.pose.pose
        self.pos, self.ori = pose_to_numpy(pose)
        self.odomSet = True

    def pcodarSubscriber(self, msg):

        if not self.odomSet:
            return

        self.scalesSet = True

        for i,object in enumerate(msg.objects):

            buoy_pos = rosmsg_to_numpy(object.pose.position)
            dist = np.linalg.norm(buoy_pos - self.pos)
            radius = 50

            scale_x = object.scale.x
            scale_y = object.scale.y
            scale_z = object.scale.z

            if i >= len(self.scale_x_avg):
                self.scale_x_avg.append(0)
                self.scale_x_count.append(0)
                self.scale_y_avg.append(0)
                self.scale_y_count.append(0)
                self.scale_z_avg.append(0)
                self.scale_z_count.append(0)

            self.scale_x_count[i] += 1
            self.scale_y_count[i] += 1
            self.scale_z_count[i] += 1

            '''
            #select highest
            if self.scale_x_avg[i] < scale_x:
                self.scale_x_avg[i] = scale_x
            if self.scale_y_avg[i] < scale_y:
                self.scale_y_avg[i] = scale_y
            if self.scale_z_avg[i] < scale_z:
                self.scale_z_avg[i] = scale_z
            '''

            self.scale_x_avg[i] = (self.scale_x_avg[i]*(self.scale_x_count[i]-1) + scale_x) / (self.scale_x_count[i])
            self.scale_y_avg[i] = (self.scale_y_avg[i]*(self.scale_y_count[i]-1) + scale_y) / (self.scale_y_count[i])
            self.scale_z_avg[i] = (self.scale_z_avg[i]*(self.scale_z_count[i]-1) + scale_z) / (self.scale_z_count[i])

            #check:
            # buoy is in front of boat
            # buoy is within a certain radius of boat
            # average buoy height is greater than threshold
            if not self.buoy_in_front_of_boat(buoy_pos) or \
                not (dist < radius) or \
                self.scale_z_avg[i] < 0.35:

                #print(i)
                #print(self.buoy_in_front_of_boat(buoy_pos))
                #print(dist)
                #print(self.scale_z_avg[i])
                #print("\n")
                continue

            if (self.closest_two_dists[0] is None or dist < self.closest_two_dists[0]) and \
                (self.closest_two_indicies[0] != i):
                
                temp = self.closest_two_indicies[0]
                self.closest_two_dists[0] = dist
                self.closest_two_indicies[0] = i

                self.closest_two_indicies[1] = temp
                continue
            
            if (self.closest_two_dists[1] is None or dist < self.closest_two_dists[1]) and \
                (self.closest_two_indicies[0] != i):
                
                self.closest_two_dists[1] = dist
                self.closest_two_indicies[1] = i
                continue
            

    #If odom and range_bearing haven't published yet then return default values of 0
    def handler(self, req):
        #[x, y, z]
        cones = TwoClosestConesResponse()
        
        if not(self.scalesSet and self.odomSet) or \
            self.closest_two_indicies[0] is None or \
            self.closest_two_indicies[1] is None:
            return cones

        print(self.closest_two_indicies)
        print(self.closest_two_dists)
        print("\n")
        cones.object_index1 = self.closest_two_indicies[0]
        cones.object_index2 = self.closest_two_indicies[1]
        
        return cones

    def buoy_in_front_of_boat(self, buoy_pos):

        #get global angle of vector between boat and buoy
        vect = [ buoy_pos[0] - self.pos[0], buoy_pos[1] - self.pos[1]]
        theta = math.atan2(vect[1], vect[0])

        #get global angle of direction boat is pointing as yaw
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.ori)

        #check relative angle, if abs(rel_angle) is less than 90deg...
        #buoy is in front of boat
        if abs(theta - yaw) < math.pi/2 or abs(theta+(2*math.pi) - yaw) < math.pi/2:
            return True
        else:
            return False

if __name__ == '__main__':
    rospy.init_node('two_closest_cones_node')
    try:
        PcodarAverageScales()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
