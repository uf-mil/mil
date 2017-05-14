from txros import util
import numpy as np
from sub8.sub_singleton import PoseSequenceCommander


@util.cancellableInlineCallbacks
def run(sub_singleton):
    print "Starting"
    poses = [np.array([-1, 0, 0, 0, 0, 3]),
    		np.array([-1, 0, 0, 0, 0, 0]),
    		np.array([-1, 0, 0, 0, 0, 3]),]
    poses_command = PoseSequenceCommander(sub_singleton,poses)
    yield poses_command.go_to_sequence()
    
    print "Done!"
