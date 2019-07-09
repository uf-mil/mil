from txros import util
import numpy as np
from mil_misc_tools import text_effects
from sub8.sub_singleton import PoseSequenceCommander

fprint = text_effects.FprintFactory(title="SEQUENCE_TEST").fprint


@util.cancellableInlineCallbacks
def run(sub_singleton):
    '''Used for testing in sim. These commands may not be safe for the real sub.'''
    fprint("Starting Test With Eulers", msg_color="blue")
    positions = [np.array([-1, 0, 0]), np.array([0, 0, 3]),
                 np.array([-1, 0, 0]), np.array([0, 0, 3])]
    orientations = [np.array([-1, 0, 0]), np.array([0, 0, 3]),
                    np.array([-1, 0, 0]), np.array([0, 0, 3])]
    poses_command = PoseSequenceCommander(sub_singleton)
    yield poses_command.go_to_sequence_eulers(positions, orientations)
    fprint("Done With Eulers!", msg_color="blue")

    fprint("Starting Test With Quaternions", msg_color="blue")
    orientations = [np.array([0, 0, 0, 1]), np.array([0, 0, 0, 1]),
                    np.array([0, 0, 0, 1]), np.array([0, 0, 0, 1])]
    yield poses_command.go_to_sequence_quaternions(positions, orientations)
    fprint("Done with Quaternions", msg_color="blue")
