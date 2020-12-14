import rospy

class LazyPublisher(rospy.Publisher):
  def __init__(self, *args, **kwargs):
    super(LazyPublisher, self).__init__(*args, **kwargs)

  def publish(self, *args):
    if self.get_num_connections() > 0:
      super(LazyPublisher, self).publish(*args)
