#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from gazebo_msgs.msg import LinkStates
import tf
from visualization_msgs.msg import MarkerArray
from mil_msgs.msg import GaussianDistribution, GaussianDistributionStamped

from distributions.gaussian import Gaussian

# My stuff
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import message_filters

base_link = 'wamv::base_link' 


links = ['robotx_navigation_challenge::gate_' + i + '::link' # list of name of links for the buoys
          for i in ['start_red',
                    'start_green',
                    'end_red',
                    'end_green']] 


g_topic = rospy.get_param('distributions')['topics']['mil_msgs.msg.GaussianDistributionStamped'] # getting topic name for gausssain


# Publishers
pub = rospy.Publisher(g_topic, numpy_msg(GaussianDistributionStamped), queue_size=10)           # Publisher to publish to Gaussian topic
debug_pub = rospy.Publisher('debug/indicated_gaussians', numpy_msg(MarkerArray), queue_size=1)  # Publisher to publish gaussian from rviz


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


# Callback Function
def cb(image, msg):

    seq[0] += 1 # increment count for callback
    cnt[0] += 1 # increment count for dataset image max

    # Only store 500 different images
    if cnt[0] < 500:
        if seq[0] % 30000 == 0: # only use each 3000th count because quick rate will overload memory if not
            #gd.header.seq += 1
            stamp = rospy.Time.now()

            # Get image from front left camera
            image_width = image.width
            image_height = image.height
            image_data = image.data   
            image_encoding = image.encoding # rgb8

            
            # Convert ROS Image to grayscale CV2 image (m,n,1)
            cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
            rows, cols = cv_image.shape

            # Create m,n,7 array. This array has 7 layers of mxn. Each pixel, mxn, has 7 values
            # |mu|, cov[0,0], cov[0,1], cov[0,2], cov[1,1], cov[1,2], cov[2,2]
            buoy_arr = np.zeros(rows,cols,7)


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

            # Iterate through each pixel in cv2 image
            for m in range(rows):
                for n in range(cols):
                    pixel = cv_image[m][n]
              
                    # go through links and transform them into the wamv's frame if match pixel
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


                        # if pixel is same as y,z in 3d (or x,y in 2d) of link than get gaussian    
                        if ((m = v[1]) and (n == v[2])):

                          # Create Gaussian
                          g = Gaussian(3, mu, cov, 'base_link') # 3 is number of dimensions
                          
                          # if the object is in front of the camera and < 100m away
                          if g.mu[0] > 0 and np.linalg.norm(g.mu) < 100:
                              #if i > 3: 
                              #  g.mu += np.array([0,0, 2])
                              marker.markers.append(g.to_rviz_marker()) # append gaussian to rviz marker array
                              marker.markers[-1].id = i
                              gd.distribution.mu = g.mu
                              gd.distribution.cov = np.ravel(g.cov)
                              #gd.distribution.id = links[i]
                              #gd.distribution.classification = links[i]
                              gd.header.seq += 1
                              gd.header.stamp = stamp
                              pub.publish(gd)

                              # Add values to output array if pixel has mu
                              buoy_arr[m,n,0] = abs(mu)
                              buoy_arr[m,n,1] = cov[0,0]
                              buoy_arr[m,n,2] = cov[0,1]
                              buoy_arr[m,n,3] = cov[0,2]
                              buoy_arr[m,n,4] = cov[1,1]
                              buoy_arr[m,n,5] = cov[1,2]
                              buoy_arr[m,n,6] = cov[2,2]
                          
                          # if pixel has no corresponding mu
                          else:
                              buoy_arr[m,n,0] = 0
                              buoy_arr[m,n,1] = 0
                              buoy_arr[m,n,2] = 0
                              buoy_arr[m,n,3] = 0
                              buoy_arr[m,n,4] = 0
                              buoy_arr[m,n,5] = 0
                              buoy_arr[m,n,6] = 0
                        
                        # if no link state then no gaussian
                        else:
                              buoy_arr[m,n,0] = 0
                              buoy_arr[m,n,1] = 0
                              buoy_arr[m,n,2] = 0
                              buoy_arr[m,n,3] = 0
                              buoy_arr[m,n,4] = 0
                              buoy_arr[m,n,5] = 0
                              buoy_arr[m,n,6] = 0
                        

                    # Save data to dataset
                    with open('buoy_dataset.dat', 'wb') as f:
                        np.savetext(f,buoy_arr)
                        f.flush()
                        sleep(0.1)


                    debug_pub.publish(marker)

    # Pickle data and exit program
    else:

        # Pickle data for export
        pickle.dump(buoy_arr, open("buoy_dataset.pickle","wb"))
        exit()




if __name__ == '__main__':
   
  rospy.init_node("create_cnn_dataset_node", anonymous=True)

  try:
    image_sub = rospy.Subscriber('/wamv/sensors/cameras/front_left_camera/image_raw', Image) 
    link_sub = rospy.Subscriber('/gazebo/link_states', numpy_msg(LinkStates))

    # time synchornizer, get data from multiple topics from same time stamp
    ts = message_filters.TimeSynchronizer([image_sub, link_sub], 1)
    ts.registerCallback(cb)

    rospy.spin()

  except rospy.ROSInterruptException:
        pass