#!/usr/bin/env python

'''this is a ros server that is made to dynamically save sensor data for run time world modeling
* the data that is saved is based on statistical ditrubtions of percieved objects from perception algorithms
*  a dimention is applied on top to each sensor space which is:
  the sum of all unique pdfs which are projectable onto that sensor space
* what is a "unique pdf"?
  when a pdf is signaled, an id (which is a str) is associated with it. If there are multiple reports of a pdf with the same pdf within a quamtum, they are multplied together to get an effective intersection of the most likely distribution on the percived object.
'''
import rospy
from rospy import numpy_msg
from sensor_spaces.sensor_space_factory import SensorSpaceFactory
from mil_msgs.srv import GaussianDistribution


class UnifiedPerceptionServer:
  def __init__(self, sensors, quantum=0.2, min_score=0.5):
    '''
    @param quantum: the amount of time to wait to consider all gradients together
    @param sensors: a list of tuples of (sensor topics, sensor space type)
    '''
    rospy.init_node('unified_perception_server')
    # create a sensor space instance for each sensor
    self.sensor_spaces = {}
    self.min_score = min_score
    self.quantum = quantum
    for i in sensors:
      self.sensor_spaces[i[0]] = SensorSpaceFactory(i[1])(i[0])
    # setup a timer that once every quantum will call the quantum callback
    # setup a ros service that will regsiter distributions in certain sensor spaces
    self.distributions = {}
    # currently the only supported distributions are gaussian, but no reason others couldn't be added later
    distribution_service = rospy.service('report_gaussian_distribution',
                                         numpy_msg(GaussianDistribution),
                                         self.register_gaussian)


  def register_gaussian(self, msg):
    if msg.id in self.distrubitions:
      self.distrubutions[msg.id].append(msg)
    else
      self.distrubutions[msg.id] = [msg]


  def quantum_callback(self):
    interesting_data = []
    # may need to parrellize this for perforamance increases
    for name, sensor in self.sensor_spaces:
      projected_dists = {}
      for key, dists in self.distributions:
        projected_dists[key] = []
        for dist in dists:
          # project the distributions onto the sensor
          projected_dists[key].append(sensor.project_distribution_from(dist))
        # reduce and add each unique distrubtuion
        sensor.add_distribution(reduce(lambda x, y : x*y, projected_dists[key])
      # extract the interesting data from that sensor
      interesting_data += sensor.extract_interesting_regions(self.min_score)
      sensor.clear_msg_buffer()
    #TODO: publish the useful data to the 'world model topic'
    return


if __name__ == '__main__':
  a = UnifiedPerceptionServer([('/camera', 'Camera')])
  rospy.spin()
