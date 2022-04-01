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
        self.max_z = []
        self.min_z = []
        self.pos = None
        self.ori = None
        self.odomSet = False
        self.scalesSet = False
        self.closest_two_dists = [None,None]
        self.closest_two_indicies = [None,None]
        self.closest_two_buoys = [None,None]
        self.buoys_passed = []
        self.average_thres = 0.4
        self.max_thres = 0.85
        self.labeled_objects = []
        self.last_gate = False

    def odometrySubscriber(self, msg):
        pose = msg.pose.pose
        self.pos, self.ori = pose_to_numpy(pose)
        self.odomSet = True

    def pcodarSubscriber(self, msg):

        self.new_closest_two_indicies = [None,None]
        self.new_closest_two_dists = [None,None]
        self.new_closest_two_buoys = [None,None]

        if not self.odomSet:
            return

        self.scalesSet = True

        #for some reason any new clusters are added to front of list
        (msg.objects).reverse()
        self.labeled_objects = msg.objects

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
                self.max_z.append(0)
                self.min_z.append(10)

            self.scale_x_count[i] += 1
            self.scale_y_count[i] += 1
            self.scale_z_count[i] += 1

            
            #select highest
            if self.max_z[i] < scale_z:
                self.max_z[i] = scale_z

            #select highest
            if self.min_z[i] > scale_z:
                self.min_z[i] = scale_z

            self.scale_x_avg[i] = (self.scale_x_avg[i]*(self.scale_x_count[i]-1) + scale_x) / (self.scale_x_count[i])
            self.scale_y_avg[i] = (self.scale_y_avg[i]*(self.scale_y_count[i]-1) + scale_y) / (self.scale_y_count[i])
            self.scale_z_avg[i] = (self.scale_z_avg[i]*(self.scale_z_count[i]-1) + scale_z) / (self.scale_z_count[i])

            if self.max_z[i] > self.max_thres and self.scale_z_avg[i] > self.average_thres:
                #print(i, self.scale_z_avg[i], self.max_z[i], self.min_z[i],dist, "cone")
                pass
            elif self.max_z[i] < self.max_thres and self.scale_z_avg[i] < self.average_thres:
                #print(i, self.scale_z_avg[i], self.max_z[i], self.min_z[i],dist, "round")
                continue
            else:
                #big waves usually cause discrepencies and cause averages to increase
                if self.scale_z_avg[i] > 0.6:
                    pass
                    #print(i, self.scale_z_avg[i], self.max_z[i], self.min_z[i],dist, "cone")
                else:
                    #print(i, self.scale_z_avg[i], self.max_z[i], self.min_z[i],dist, "round")
                    continue

            #check:
            # buoy is in front of boat
            # buoy is within a certain radius of boat
            if not self.buoy_in_front_of_boat(buoy_pos) or \
                not (dist < radius):

                continue

            if (self.new_closest_two_dists[0] is None or dist < self.new_closest_two_dists[0]) and \
                (self.new_closest_two_indicies[0] != i) and (i not in self.buoys_passed):
                
                temp_index = self.new_closest_two_indicies[0]
                temp_buoy = self.new_closest_two_buoys[0]
                temp_dist = self.new_closest_two_dists[0]


                self.new_closest_two_dists[0] = dist
                self.new_closest_two_buoys[0] = object.pose.position
                self.new_closest_two_indicies[0] = i

                self.new_closest_two_indicies[1] = temp_index
                self.new_closest_two_buoys[1] = temp_buoy
                self.new_closest_two_dists[1] = temp_dist
                continue
            
            if (self.new_closest_two_dists[1] is None or dist < self.new_closest_two_dists[1]) and \
                (self.new_closest_two_indicies[0] != i) and (i not in self.buoys_passed):
                
                self.new_closest_two_dists[1] = dist
                self.new_closest_two_buoys[1] = object.pose.position
                self.new_closest_two_indicies[1] = i
                continue
        
        self.closest_two_buoys = self.new_closest_two_buoys
        self.closest_two_dists = self.new_closest_two_dists
        self.closest_two_indicies = self.new_closest_two_indicies

    #If odom and range_bearing haven't published yet then return default values of 0
    def handler(self, req):
        #[x, y, z]
        cones = TwoClosestConesResponse()

        index1 = self.closest_two_indicies[0]
        index2 = self.closest_two_indicies[1]

        if not(self.scalesSet and self.odomSet) or \
            index1 is None or \
            index2 is None or \
            self.last_gate or \
            "round" in self.labeled_objects[index1].labeled_classification or \
            "round" in self.labeled_objects[index2].labeled_classification:

            cones.no_more_buoys = True
            return cones

        if self.labeled_objects[index1].labeled_classification == "mb_marker_buoy_black" or \
            self.labeled_objects[index2].labeled_classification == "mb_marker_buoy_black":
            self.last_gate = True

        cones.no_more_buoys = False
        cones.object1 = self.closest_two_buoys[0]
        cones.object2 = self.closest_two_buoys[1]

        self.buoys_passed.append(self.closest_two_indicies[0])
        self.buoys_passed.append(self.closest_two_indicies[1])
        
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
        
