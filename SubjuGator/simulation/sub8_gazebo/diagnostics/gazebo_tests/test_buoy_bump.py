import txros
from twisted.internet import defer
import numpy as np
from diagnostics.gazebo_tests import common
from sub8 import pose_editor
from missions import buoy


# TODO: Allow generic names
class Job(common.Job):
    _job_name = 'test_buoy_bump'

    @txros.util.cancellableInlineCallbacks
    def setup(self):
        print 'setting up'
        buoy_loc = np.array([20.0, 20.0, -1.0])
        start_seed = np.array([15.0, 20.0, -2.0])

        pe = pose_editor.PoseEditor('/map', start_seed, np.array([0.0, 0.0, 0.0, 1.0]))
        pe = pe.look_at_without_pitching(buoy_loc)
        pe = pe.backward(2.0)
        pe = pe.relative(
            (np.random.random(3) - 0.5) * [2.5, 2.5, 2.0]
        )
        yield self.set_model_pose(pe.as_Pose())
        yield self.nh.sleep(1.0)

    @txros.util.cancellableInlineCallbacks
    def run(self, sub):
        print 'running'
        # yield sub.move.right(2.0).go()
        # yield sub.move.down(1.0).go()
        # yield sub.move.left(2.0).go()
        # yield sub.move.up(1.0).go()

        yield buoy.run(sub)

        success = True
        reason = "No reason"
        yield defer.returnValue((success, reason))
