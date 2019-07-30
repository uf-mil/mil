from .sub_singleton import SubjuGator
from txros import util
import numpy as np

SIDE_LENGTH = 1  # meters
SPEED_LIMIT = .2  # m/s


class Square(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
        yield self.plane_sonar.get_group_of_points(np.array([1,0,0])) 
        center = self.move.forward(0).zero_roll_and_pitch()
        for i in range(4):
            forward = self.move.forward(SIDE_LENGTH).zero_roll_and_pitch()
            right = forward.right(SIDE_LENGTH).zero_roll_and_pitch()

            yield forward.go(speed=SPEED_LIMIT)
            yield right.go(speed=SPEED_LIMIT)
            yield forward.go(speed=SPEED_LIMIT)
            yield center.go(speed=SPEED_LIMIT)
            center = center.yaw_right_deg(90)
            yield center.go(speed=SPEED_LIMIT)

        print "Done!"
