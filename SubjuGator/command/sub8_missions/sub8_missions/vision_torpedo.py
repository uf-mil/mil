#simple script to test if image recognition will shoot torpedo since we cannot test it using thrusters without odom

from .sub_singleton import SubjuGator
from txros import util
import rospy
import mil_ros_tools
from sub8_msgs.msg import PathPoint
from mil_misc_tools import text_effects

fprint = text_effects.FprintFactory(title="VISION_TORPEDO", msg_color="cyan").fprint
SIDE_LENGTH = 1  # meters
SPEED_LIMIT = .2  # m/s


class VisionTorpedo(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
      done = False
      
      move_info_sub = yield self.nh.subscribe(
        '/move_to_badge', PathPoint)

      while done == False:
        #listen to topic move where it tells us to move
        move_info = yield move_info_sub.get_next_message()

        #we have eyes on the target
        if move_info.yaw == 2.0:
          fprint("FIRE")
          yield self.actuators.shoot_torpedo1()
          yield self.nh.sleep(1)
          yield self.actuators.shoot_torpedo2()
          done = True

      fprint("Done!")
