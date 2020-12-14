#!/usr/bin/env python

'''this is a ros server that is made to dynamically save sensor data for run time world modeling
* the data that is saved is based on statistical ditrubtions of percieved objects from perception algorithms
*  a dimention is applied on top to each sensor space which is:
  the sum of all unique pdfs which are projectable onto that sensor space
* what is a "unique pdf"?
  when a pdf is signaled, an id (which is a str) is associated with it. If there are multiple reports of a pdf
    with the same pdf within a quamtum, they are multplied together to get an effective intersection of the most likely distribution on the percived object.
'''
import rospy
from rospy.numpy_msg import numpy_msg
from threading import Lock, Thread
import geometry_msgs
import sensor_msgs
import visualization_msgs
from visualization_msgs.msg import MarkerArray

from sensor_spaces.sensor_space_factory import SensorSpaceFactory
import mil_msgs
from mil_msgs.msg import WorldModel, WorldModelStamped, \
                         GaussianDistribution, GaussianDistributionStamped



from distributions.gaussian import Gaussian
from mil_ros_tools import LazyPublisher


class UnifiedPerceptionServer:
  def __init__(self):
    self.inited = False
    self.id = 0
    rospy.init_node('unified_perception_server')
    # create a sensor space instance for each sensor
    self.sensors = {}
    self.min_score = rospy.get_param('min_score')
    self.world_frame = rospy.get_param('world_frame_id')
    sensors = rospy.get_param('sensors')
    self.debug_pubs = {'sensors' : {}, 'distributions': {}}
    for name, values in sensors.items():
      self.sensors[name] = SensorSpaceFactory(values['sensor_space'])(name)
      self.debug_pubs['sensors'][name] = LazyPublisher(values['visualization']['topic'],
                                       eval(values['visualization']['msg_type']),
                                       queue_size=10)
    self.distributions = {}
    distributions = rospy.get_param('distributions')
    self.subs = [rospy.Subscriber(j, numpy_msg(eval(i)), self.distribution_cb, 10)
                 for i, j in distributions['topics'].items()]
    self.distribution_debug_types = {}
    for dist_type, vis in distributions['visualization'].items():
      self.debug_pubs['distributions'][str(eval(dist_type))] = LazyPublisher(vis['topic'],
                                                      eval(vis['vis_msg_type']), queue_size=10)
    self.world_model_pub = rospy.Publisher(rospy.get_param('world_model_topic'),
                                           WorldModelStamped, queue_size=10)
    self.timer = rospy.Timer(rospy.Duration(rospy.get_param('quantum')), self.quantum_cb)
    _msg_type_to_world_field = \
      rospy.get_param('sensor_msg_type_to_world_model_field')
    self.msg_type_to_world_field = {str(eval(i)): j
                                    for i, j in _msg_type_to_world_field.items()}
    self.cov_growth_rate = rospy.get_param('cov_growth_rate')
    self.lock = Lock()
    self.inited = True


  def distribution_cb(self, msg, count):
    if self.inited != True:
      return
    # project from sensor space to the world space
    dist = self.sensors[msg.distribution.sensor_name].project_to_world_frame(msg)
    # add to the distributions dict
    if msg.distribution.id != '':
      if msg.distribution.id in self.distributions:
        self.distributions[msg.distribution.id] *= dist
      else:
        self.distributions[msg.distribution.id] = dist
    else:
      self.distrubutions[str(self.id)] = dist
      self.id += 1
    self.lock.acquire()
    # apply distributions to the data in the buffers
    threads = []
    for sensor in self.sensors.values():
      threads.append(Thread(target=sensor.apply_distributions,
                            args=[self.min_score, self.distributions]))
      threads[-1].start()
    for t in threads:
      t.join()
    for sensor_name in self.sensors:
      self.debug_pubs['sensors'][sensor_name].publish(
        self.sensors[sensor_name].edited_msg_buffer[-1])
    self.lock.release()
    return

  def quantum_cb(self, event):
    # may need to remove old distributions that are not valid anymore
    world_model = WorldModel()
    self.lock.acquire()
    for name, sensor in self.sensors.items():
      if str(sensor.msg_type) not in self.msg_type_to_world_field:
        raise Exception(str(sensor.msg_type) +
          ' does not have a corresponding field in the WorldModel msg. Specify one in the ' +
          '"sensor_msg_type_to_world_model_field" ros parameter')
      field = self.msg_type_to_world_field[str(sensor.msg_type)]
      for i in sensor.edited_msg_buffer:
        eval('world_model.' + field).append(i)
      sensor.clear_edited_msg_buffer()
    self.lock.release()
    # publish that data along with the distributions as the world model
    wms = WorldModelStamped()
    wms.world_model = world_model
    wms.header.frame_id = self.world_frame
    wms.header.stamp = rospy.Time.now()
    distribution_debug_msgs = {name: MarkerArray() for name in self.debug_pubs['distributions']}

    i = 0
    for name, dist in self.distributions.items():
      if isinstance(dist, Gaussian):
        if dist.n == 3:
          world_model.gaussians.append(dist.to_msg())
          marker = dist.to_rviz_marker()
          marker.id = i
          i += 1
          distribution_debug_msgs[str(Gaussian)].markers.append(marker)
        dist.cov *= self.cov_growth_rate
    self.world_model_pub.publish(wms)
    for msg_type, msg in distribution_debug_msgs.items():
      self.debug_pubs['distributions'][msg_type].publish(msg)

if __name__ == '__main__':
  a = UnifiedPerceptionServer()
  rospy.spin()
