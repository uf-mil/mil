import txros
from diagnostics.gazebo_tests import common


# TODO: Allow generic names
class Job(common.Job):
    _job_name = 'test_simple_motion'

    @txros.util.cancellableInlineCallbacks
    def setup(self):
        print 'setting up'
        yield self.set_model_position((0.0, 0.0, 0.0))
        yield self.nh.sleep(5)

    @txros.util.cancellableInlineCallbacks
    def run(self, sub):
        print 'running'
        yield None
