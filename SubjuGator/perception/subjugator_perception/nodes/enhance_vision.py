#!/usr/bin/env python3

# IMPORTS -- Underwater Enhancement ML 

import os
import torch
import numpy as np
from PIL import Image
from model import PhysicalNN
import argparse
from torchvision import transforms
import datetime
import math

# IMPORTS -- ROS vision

import rospy
import tf
from image_geometry import PinholeCameraModel
from mil_ros_tools import (
    Image_Publisher,
    Image_Subscriber,
)
from std_srvs.srv import SetBool, SetBoolResponse
from subjugator_msgs.srv import (
    VisionRequest,
    VisionRequest2D,
    VisionRequest2DResponse,
    VisionRequestResponse,
)

class EnhanceVision:

    def __init__(self):
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Load model
        model = PhysicalNN()
        model = torch.nn.DataParallel(model).to(device)
        checkpoint = torch.load('checkpoints/model_best_2842.pth.tar')
            
        print("In Progress...")
        
if __name__== "__main__":
    rospy.init_node("enhance_vision")
    EnhanceVision()
    rospy.spin()
