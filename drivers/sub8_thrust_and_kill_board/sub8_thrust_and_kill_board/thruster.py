from numpy import polyval
from numpy import clip


def make_thruster_dictionary(dictionary):
    '''
    Make a dictionary mapping thruster names to Thruster objects
    '''
    ret = {}
    for thruster, content in dictionary.iteritems():
        ret[thruster] = Thruster.from_dict(content)
    return ret


class Thruster(object):
    '''
    Models the force (thrust) to PWM (effort) of a thruster
    '''
    def __init__(self, forward_calibration, backward_calibration):
        self.forward_calibration = forward_calibration
        self.backward_calibration = backward_calibration

    @classmethod
    def from_dict(cls, data):
        forward_calibration = data['calib']['forward']
        backward_calibration = data['calib']['backward']
        return cls(forward_calibration, backward_calibration)

    def effort_from_thrust_unclipped(self, thrust):
        if thrust < 0:
            return polyval(self.backward_calibration, thrust)
        else:
            return polyval(self.forward_calibration, thrust)

    def effort_from_thrust(self, thrust):
        unclipped = self.effort_from_thrust_unclipped(thrust)
        return clip(unclipped, -5., 5.)
