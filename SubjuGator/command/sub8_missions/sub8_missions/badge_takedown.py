from .sub_singleton import SubjuGator
from txros import util
import rospy
import mil_ros_tools
from sub8_msgs.msg import PathPoint
from mil_misc_tools import text_effects

fprint = text_effects.FprintFactory(title="BADGE_TAKEDOWN", msg_color="cyan").fprint
SIDE_LENGTH = 1  # meters
SPEED_LIMIT = .2  # m/s


class BadgeTakedown(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
      done = False

      forward = self.move.forward(SIDE_LENGTH).zero_roll_and_pitch()
      right = forward.right(SIDE_LENGTH).zero_roll_and_pitch()
      left = forward.left(SIDE_LENGTH).zero_roll_and_pitch()

      move_info_sub = yield self.nh.subscribe(
        '/move_to_badge', PathPoint)

      while done == False:
        #listen to topic move where it tells us to move
        move_info = yield move_info_sub.get_next_message()

        #we do not see the badge
        if move_info.yaw == 0.0:
          yield self.move.yaw_right_deg(10.0).go(speed=SPEED_LIMIT)

        #we have a vector
        elif move_info.yaw == 1.0:
           pose = move_info.position
           vec = [pose.x, pose.y, pose.z]
           yield self.move.relative(vec).zero_roll_and_pitch().go(speed=SPEED_LIMIT)

        elif move_info.yaw == 2.0:
          fprint("We have arrived")
          done = True

        '''
        #we have eyes on the target
        elif move_info.yaw == 2.0
          fprint("FIRE")
          yield self.actuators.shoot_torpedo1()
          yield self.actuators.sheet_torpedo2()
          done = True
        '''

      fprint("Done!")
