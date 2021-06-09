from .sub_singleton import SubjuGator
from txros import util
import rospy
import mil_ros_tools
from sub8_msgs.msg import ThrusterCmd
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
        '/move_to_badge', ThrusterCmd)

      while done == False:
        #listen to topic move where it tells us to move
        move_info = yield move_info_sub.get_next_message()

        #moving to location
        if move_info.name == "ROTATE_RIGHT":
          yield self.move.yaw_right_deg(move_info.thrust).go(speed=SPEED_LIMIT)
        elif move_info.name == "ROTATE_LEFT":
          yield self.move.yaw_left_deg(move_info.thrust).go(speed=SPEED_LIMIT)
        elif move_info.name == "FORWARD":
          yield self.move.forward(move_info.thrust).go(speed=SPEED_LIMIT)
        elif move_info.name == "LEFT":
          yield self.move.left(move_info.thrust).go(speed=SPEED_LIMIT)
        elif move_info.name == "RIGHT":
          yield self.move.right(move_info.thrust).go(speed=SPEED_LIMIT)
        elif move_info.name == "BACKWARD":
          yield self.move.backward(move_info.thrust).go(speed=SPEED_LIMIT)
        elif move_info.name == "UP":
          yield self.move.up(move_info.thrust).go(speed=SPEED_LIMIT)
        elif move_info.name == "DOWN":
          yield self.move.down(move_info.thrust).go(speed=SPEED_LIMIT)
        elif move_info.name == "SHOOT":
          fprint("FIRE")
          yield self.actuators.shoot_torpedo1()
          yield self.actuators.shoot_torpedo2()
          done = True
        elif move_info.name == "ARRIVED":
          fprint("We have arrived")
          done = True
        else:
          yield self.move.yaw_right_deg(45).go(speed=SPEED_LIMIT)

      fprint("Done!")
