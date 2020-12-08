from abc import ABCMeta, abstractmethod
from threading import Thread, Lock
import tf
import rospy


class SensorSpace():
  __metaclass__ = ABCMeta
  def __init__(self, name):
    # should be a list of tuples of (msg, mutex)
    self.msg_buffer = []
    self.name = name
    self.listener = tf.TransformListener()
    self.world_frame = rospy.get_param('world_frame_id')

  def clear_msg_buffer(self):
    self.musg_buffer = []

  @abstractmethod
  def project_to_world_frame(msg):
    pass

  @abstractmethod
  def project_from_world_frame(dist):
    pass

  @abstractmethod
  def apply_distributions_to(self, (msg, mutex), dists, min_score):
    pass

  def apply_distributions(self, min_score, distributions):
    dists = {}
    for name, dist in distributions.items():
      dists[name] = self.project_from_world_frame(dist)
    # call extract_interesting_region in a new thread on each msg in the message list
    threads = []
    for i in self.msg_buffer:
      threads.append(Thread(target=self.apply_distributions_to,
                            args=[i, dists, min_score]))
      threads[-1].start()
    # wait for them to finish
    for i in threads:
      i.join()
    return

  def callback(self, msg):
    self.msg_buffer.append((msg, Lock()))
