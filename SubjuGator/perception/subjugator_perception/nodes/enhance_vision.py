#!/usr/bin/env python3

# ML MODEL CITATION:
""" 
@misc{chen2021underwater,
  title={Underwater Image Enhancement based on Deep Learning and Image Formation Model}, 
  author={Xuelei Chen and Pin Zhang and Lingwei Quan and Chao Yi and Cunyue Lu},
  year={2021},
  eprint={2101.00991},
  archivePrefix={arXiv},
  primaryClass={eess.IV} 

"""

# IMPORTS -- Underwater Enhancement ML 

import os
import torch
import numpy as np
from PIL import Image
from model import PhysicalNN
from torchvision import transforms
import datetime

# IMPORTS -- ROS vision

import rospy
from image_geometry import PinholeCameraModel
from mil_ros_tools import (
    Image_Publisher,
    Image_Subscriber,
)

class EnhanceVision:
    # Currently running on CPU, needs to change in the future.

     #device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
     device = torch.device('cpu')

    # Load model
     model = PhysicalNN()
     model = torch.nn.DataParallel(model).to(device)
     checkpoint = torch.load('checkpoints/model_best_2842.pth.tar', map_location=device)
     model.load_state_dict(checkpoint['state_dict'])
     print("=> loaded model at epoch {}".format(checkpoint['epoch']))
     model = model.module
     model.eval()

     testtransform = transforms.Compose([
             transforms.ToTensor(),
          ])
     unloader = transforms.ToPILImage()

     def __init__(self):
        camera = rospy.get_param("~image_topic", "/camera/down/image_rect_color")

        self.image_sub = Image_Subscriber(camera, self.enhance_callback)
        self.camera_info = self.image_sub.wait_for_camera_info()
        assert self.camera_info is not None
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(self.camera_info)
        self.image_pub = Image_Publisher("~enhance_cam_topic")

     def enhance_callback(self, msg):
        # Create Image from array
        starttime = datetime.datetime.now()
        img = Image.fromarray(msg.astype('uint8'), 'RGB')
        inp = self.testtransform(img).unsqueeze(0)
        inp = inp.to(self.device)
        out = self.model(inp)
       
        # Place result images in directory
        corrected = self.unloader(out.cpu().squeeze(0))
        print(type(corrected))

        self.image_pub.publish(np.array(corrected))

        # The commented code is to create a file with a png image of the captured img data.

        #dir = '{}/results_{}'.format('.', self.checkpoint['epoch'])
        #if not os.path.exists(dir):
        #    os.makedirs(dir)
        #corrected.save(dir+'/{}corrected.png'.format("camera_"))
        endtime = datetime.datetime.now()
        print(endtime-starttime)
        
        
if __name__== "__main__":
    rospy.init_node("enhance_vision")
    EnhanceVision()
    rospy.spin()
