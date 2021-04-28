#!/usr/bin/env python


# create_dataset.py
#
# Purpose:  This program will create a dataset of 3D arrays in a csv that will be used in a CNN.
#           The data saved will be |mu| or the distance to the buoy, and the upper triangle of the 
#           covariance matrix per each pixel in each frame. 
#
# Usage: 'roslaunch navigator_launch vrx.launch' , 'roslaunch unified perception create_dataset.launch'
#         By looking at the terminal window, a print statement saying "Done writing dataset to CSV" will 
#         be displayed. To change the number of frames saved to the dataset, change num_frames. To change 
#         where the  file is stored, alter file_name. This file can be very large so make sure to save it
#         somewhere appropriately.
#
# 
#
# Next Step: Use as many frame as needed  to train a CNN model that can be used. 



import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from gazebo_msgs.msg import LinkStates
import tf
from visualization_msgs.msg import MarkerArray
from mil_msgs.msg import GaussianDistribution, GaussianDistributionStamped
from distributions.gaussian import Gaussian
from cv_bridge import CvBridge
import message_filters
import csv
from sensor_spaces.camera import Camera
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from sensor_spaces.sensor_space import SensorSpace


base_link = 'wamv::base_link' 

# Create list of link names for buoys
links = ['robotx_navigation_challenge::gate_' + i + '::link'
          for i in ['start_red',
                    'start_green',
                    'end_red',
                    'end_green']] 

# Init node
rospy.init_node("create_cnn_dataset_node")

# Create GaussianDistrutionStamped Message
gd = GaussianDistributionStamped()
gd.header.seq = 0
gd.header.frame_id = 'base_link'
gd.distribution.sensor_name = 'front_left_camera'

# Create instance of Image message to grab information
image = Image()

# Create instance of cv_bridge
bridge = CvBridge()

# Seq will keep track of how many times callback is called
seq = [0] 

# Cnt will keep track of how many times data is stored to output
cnt = [0]

# This is the number of frames that will be saved to the dataset
num_frames = 2

# dataset file name
file_name = '/home/hadrien/buoy_dataset.csv'

# Information needed to create camera model and 3d projection
info_topic = '/wamv/sensors/cameras/front_left_camera/camera_info'
info = rospy.wait_for_message(info_topic, CameraInfo)
model = PinholeCameraModel()
model.fromCameraInfo(info)
o_frame = model.tfFrame()
optical_frame = o_frame[o_frame.rfind('/')+1:]
listener = tf.TransformListener()

debug_pub = rospy.Publisher('debug/indicated_gaussians', numpy_msg(MarkerArray), queue_size=1) # publich gaussian from rviz


# Callback Function
def cb(image, msg):

    seq[0] += 1 # increment count for callback

    if seq[0] % 30 == 0: # only use each 300th count because quick rate will overload memory if not
        
      # Create m,n,7 array. This array has 7 layers of mxn. Each pixel, mxn, has 7 values
      # |mu|, cov[0,0], cov[0,1], cov[0,2], cov[1,1], cov[1,2], cov[2,2]
      buoy_arr = np.zeros((1280,720,7))
      
      gd.header.seq += 1
      stamp = rospy.Time.now()
      
      # Convert ROS Image to grayscale CV2 image (m,n,1)
      cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
      rows, cols = cv_image.shape

      # get the base link pose and the bouys' pose
      base_link_idx = msg.name.index(base_link)
      trans = np.array([-msg.pose[base_link_idx].position.x, # translation
                      -msg.pose[base_link_idx].position.y,
                      -msg.pose[base_link_idx].position.z])
      quat = np.array([msg.pose[base_link_idx].orientation.x, # quaternian
                      msg.pose[base_link_idx].orientation.y,
                      msg.pose[base_link_idx].orientation.z,
                      -msg.pose[base_link_idx].orientation.w])


      links_idx = [msg.name.index(i) for i in links]
      poses = [msg.pose[i] for i in links_idx]
      marker = MarkerArray()

      # go through links and transform them into the wamv's frame
      for i in range(len(links)):
        pos = poses[i].position
        v = np.array([pos.x +  trans[0], pos.y + trans[1], pos.z + trans[2]]) # apply offset
        v_u = np.hstack((tf.transformations.unit_vector(v), np.zeros(1,))) # create quaternion
        quat_p = tf.transformations.quaternion_conjugate(quat)
        mu = np.linalg.norm(v) * tf.transformations.quaternion_multiply(
                                tf.transformations.quaternion_multiply(quat, v_u),
                                quat_p)[:3] #linald is distance... all of this gives position in respect to eachother
    
        # Create covariance
        cov = np.array([[1,0,0],
                        [0,1,0],
                        [0,0,1]])

        # Create Gaussian
        g = Gaussian(3, mu, cov, 'base_link') # 3 is number of dimensions
        
        # if the object is in front of the camera and < 100m away
        if g.mu[0] > 0 and np.linalg.norm(g.mu) < 100:
  
          # Project from world frame
          g_pixel = g.transform_to_frame(optical_frame, listener, time=rospy.Time.now())[0]
          if g_pixel.mu[2] < 0:
            return

          # project mu onto the camera pixel coordinates (u,v)
          mu_pix = np.array(model.project3dToPixel(g_pixel.mu))
        
          m = int(mu_pix[0])  # save pixel coordinates from 3D projection
          n = int(mu_pix[1])

          # Save data to array for dataset
          buoy_arr[m,n,0] = np.linalg.norm(g.mu)
          buoy_arr[m,n,1] = cov[0,0]
          buoy_arr[m,n,2] = cov[0,1]
          buoy_arr[m,n,3] = cov[0,2]
          buoy_arr[m,n,4] = cov[1,1]
          buoy_arr[m,n,5] = cov[1,2]
          buoy_arr[m,n,6] = cov[2,2]

          # Test implementation heere by overlaying gaussian on image
          g_pixel = Gaussian(3,g.mu,cov,'base_link')
          marker.markers.append(g_pixel.to_rviz_marker()) # append gaussian to rviz marker array
          marker.markers[-1].id = i
          gd.distribution.cov = np.ravel(g.cov)
          gd.header.seq += 1
          gd.header.stamp = stamp

      debug_pub.publish(marker)
      

      if cnt[0] == 0:
        # convert 3d array to 2d array and save to dataset file
        buoy_arr_list = buoy_arr.tolist()
        with open(file_name, 'w') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerows(buoy_arr_list)
        #rospy.loginfo("Done writing dataset to CSV")

      elif cnt[0] < num_frames:
        # convert 3d array to 2d array and append to dataset file
        buoy_arr_list = buoy_arr.tolist()
        with open(file_name, 'a') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerows(buoy_arr_list)

        rospy.loginfo("Done writing dataset to CSV")
        image_sub.unregister()
        link_sub.unregister()

      cnt[0] += 1 # increment count for dataset image max
   



image_sub = message_filters.Subscriber('/wamv/sensors/cameras/front_left_camera/image_raw', Image) 
link_sub = message_filters.Subscriber('/gazebo/link_states', numpy_msg(LinkStates))

# time synchornizer, get data from multiple topics from same time stamp
ts = message_filters.ApproximateTimeSynchronizer([image_sub, link_sub], 10, 0.1, allow_headerless=True )
ts.registerCallback(cb)

rospy.spin()

