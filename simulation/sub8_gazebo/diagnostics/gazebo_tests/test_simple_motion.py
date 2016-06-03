import txros
from twisted.internet import defer
import numpy as np
from diagnostics.gazebo_tests import common


# TODO: Allow generic names
class Job(common.Job):
    _job_name = 'test_simple_motion'

    @txros.util.cancellableInlineCallbacks
    def setup(self):
        print 'setting up'
        xy = (np.random.random(2) - 0.5) * 20
        yield self.set_model_position(np.hstack([xy, -5]))

    @txros.util.cancellableInlineCallbacks
    def run(self, sub):
        print 'running'
        distance = 5.0

        initial_position = sub.pose.position
        yield sub.move.forward(distance).go()
        position_after_action = sub.pose.position
        yield self.nh.sleep(3.0)
        position_after_waiting = sub.pose.position

        motion_posthoc = (np.linalg.norm(position_after_waiting - position_after_action))
        distance_to_target = (np.linalg.norm(position_after_waiting - initial_position) - distance)

        # Pretty generous
        small_posthoc_motion = motion_posthoc < 0.3
        reached_target = distance_to_target < 0.1

        success = (small_posthoc_motion and reached_target)
        reason = ""
        reason += "\nMoved {}m after reporting completion".format(motion_posthoc)
        reason += "\nEnded {}m away from target".format(distance_to_target)

        yield defer.returnValue((success, reason))
