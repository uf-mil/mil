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

        small_posthoc_motion = (np.linalg.norm(position_after_waiting - position_after_action)) < 0.3
        reached_target = (np.linalg.norm(position_after_waiting - initial_position) - distance) < 0.1

        yield defer.returnValue(small_posthoc_motion and reached_target)
