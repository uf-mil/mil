#!/usr/bin/env python
#this is a cameron problem https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/
import rospy
from nav_msgs.msg import Odometry
from navigator_msgs.msg import ScaleObject
from navigator_msgs.srv import AverageScale, AverageScaleResponse
from mil_msgs.msg import PerceptionObjectArray
import math
import numpy as np

#Defines a service that returns the position of the acoustic beacon from odom and the range_bearing topic
class PcodarAverageScales():
    def __init__(self):

        #create service server
        self.server = rospy.Service('/pcodar_average_scale_service', AverageScale, self.handler)
        
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
        self.odomSet = False
        self.scalesSet = False



    def odometrySubscriber(self, msg):
        self.pos = msg.pose.pose.position
        self.odomSet = True

    def pcodarSubscriber(self, msg):

        if not self.odomSet:
            return

        self.scalesSet = True

        for i,object in enumerate(msg.objects):
            boat_pos = self.pos
            boat_pos_np = np.array([boat_pos.x, boat_pos.y, boat_pos.z])
            buoy_pos = object.pose.position
            buoy_pos_np = np.array([buoy_pos.x, buoy_pos.y, buoy_pos.z])
            dist = np.linalg.norm(buoy_pos_np - boat_pos_np)
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
            self.scale_x_avg[i] = (self.scale_x_avg[i]*(self.scale_x_count[i]-1) + scale_x) / (self.scale_x_count[i])
            self.scale_y_avg[i] = (self.scale_y_avg[i]*(self.scale_y_count[i]-1) + scale_y) / (self.scale_y_count[i])
            self.scale_z_avg[i] = (self.scale_z_avg[i]*(self.scale_z_count[i]-1) + scale_z) / (self.scale_z_count[i])

    #If odom and range_bearing haven't published yet then return default values of 0
    def handler(self, req):
        #[x, y, z]
        average_scales = AverageScaleResponse()
        
        if not(self.scalesSet and self.odomSet):
            print(self.scalesSet)
            print(self.odomSet)
            return average_scales

        scales_list = []
        for i in range(len(self.scale_x_avg)):
            scales_list.append(ScaleObject())

        for i,scales in enumerate(scales_list):
            
            scales.scale_x = self.scale_x_avg[i]
            scales.scale_y = self.scale_y_avg[i]
            scales.scale_z = self.scale_z_avg[i]

        average_scales.objects = scales_list
        
        return average_scales


if __name__ == '__main__':
    rospy.init_node('pcodar_average_scales')
    try:
        PcodarAverageScales()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
