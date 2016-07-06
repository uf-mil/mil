#!/usr/bin/python
from __future__ import division
import numpy as np
import numpy.linalg as la
from scipy import optimize
from itertools import combinations

from sub8_msgs.srv import Sonar, SonarResponse


class Multilaterator(object):
    '''
    Finds the origin location of a pulse given differential times of 
    arrival to the individual sensors. c is the wave speed in the medium of operation.
    Units:
        Hydrohone coordinates are expected in millimeters, pulse location will be given in millimeters.
        Timestamps are expected in microseconds. c is expected in millimeters per microsecond
    Note:
        hydrophone locations should be the dict returned by rospy.get_param('~/<node name>/hydrophones
    '''
    def __init__(self, hydrophone_locations, c, method):  # speed in millimeters/microsecond
        self.hydrophone_locations = []
        for key in hydrophone_locations:
            sensor_location = np.array([hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
            self.hydrophone_locations += [sensor_location]
        self.pairs = list(combinations(range(len(hydrophone_locations)),2))
        self.c = c
        self.method = method
        print "\x1b[32mSpeed of Sound (c):", self.c, "millimeter/microsecond\x1b[0m"

    def getPulseLocation(self, timestamps, method=None):
        '''
        Returns a ros message with the location and time of emission of a pinger pulse.
        '''
        if method == None:
            method = self.method
        # print "\x1b[32mMultilateration algorithm:", method, "\x1b[0m"
        assert len(self.hydrophone_locations) == len(timestamps)
        source = None
        if method == 'bancroft':
            source = self.estimate_pos_bancroft(timestamps)
        elif method == 'LS':
            source = self.estimate_pos_LS(timestamps)
        else:
            print method, "is not an available multilateration algorithm"
            return
        response = SonarResponse()
        response.x = source[0]
        response.y = source[1]
        response.z = source[2]
        print "Reconstructed Pulse:\n\t" + "x: " + str(response.x) + " y: " + str(response.y) \
            + " z: " + str(response.z) + " (mm)"
        return response

    def estimate_pos_bancroft(self, reception_times):
        N = len(reception_times)
        assert N >= 4
        
        L = lambda a, b: a[0]*b[0] + a[1]*b[1] + a[2]*b[2] - a[3]*b[3]
        
        def get_B(delta):
            B = np.zeros((N, 4))
            for i in xrange(N):
                B[i] = np.concatenate([self.hydrophone_locations[i]/(self.c), [-reception_times[i]]]) + delta
            return B
        
        delta = min([.1*np.random.randn(4) for i in xrange(10)], key=lambda delta: np.linalg.cond(get_B(delta)))
        # delta = np.zeros(4) # gives very good heading for noisy timestamps, although range is completely unreliable

        B = get_B(delta)
        a = np.array([0.5 * L(B[i], B[i]) for i in xrange(N)])
        e = np.ones(N)
        
        Bpe = np.linalg.lstsq(B, e)[0]
        Bpa = np.linalg.lstsq(B, a)[0]
        
        Lambdas = quadratic(
            L(Bpe, Bpe),
            2*(L(Bpa, Bpe) - 1),
            L(Bpa, Bpa))
        if not Lambdas: 
            return [0, 0, 0]
        
        res = []
        for Lambda in Lambdas:
            u = Bpa + Lambda * Bpe
            position = u[:3] - delta[:3]
            time = u[3] + delta[3]
            if any(reception_times[i] < time for i in xrange(N)): continue
            res.append(position*self.c)
        if len(res) == 1:
            source = res[0]
        elif len(res) == 2:
            source = [x for x in res if x[2] < 0]   # Assume that the source is below us
            if not source: 
                source = res[0]
            else:
                source = source[0]
        else:
            source = [0, 0, 0]
        return source

    def estimate_pos_LS(self, timestamps):
        '''
        Returns a ros message with the location and time of emission of a pinger pulse.
        '''
        self.timestamps = timestamps
        init_guess = np.random.normal(0,100,3)
        opt = {'disp': 0}
        opt_method = 'Powell'
        result = optimize.minimize(self.cost_LS, init_guess, method=opt_method, options=opt, tol=1e-15)
        if(result.success):
            source = [result.x[0], result.x[1], result.x[2]]
        else:
            source = [0, 0, 0]
        return source

    def cost_LS(self, potential_pulse):
        '''
        Compares the difference in observed and theoretical difference in time of arrival
        between tevery unique pair of hydrophones.

        Note: the result will be along the direction of the heading but not at the right distance.
        '''
        cost = 0
        t = self.timestamps
        c = self.c
        x = np.array(potential_pulse)
        for pair in self.pairs:
            h0 = self.hydrophone_locations[pair[0]]
            h1 = self.hydrophone_locations[pair[1]]
            residual = la.norm(x-h0) - la.norm(x-h1) - c*(t[pair[0]] - t[pair[1]])
            cost += residual**2
        return cost

    def cost_LS2(self, potential_pulse):
        """
        Slightly less accurate than the one above in terms of heading but much faster.
        """
        cost = 0
        t = self.timestamps
        x0 = self.hydrophone_locations[0][0]
        y0 = self.hydrophone_locations[0][1]
        z0 = self.hydrophone_locations[0][2]
        x = potential_pulse[0]
        y = potential_pulse[1]
        z = potential_pulse[2]
        d0 = np.sqrt((x0 - x)**2 + (y0 - x)**2 + (z0 - x)**2)
        for i in xrange(1, len(self.hydrophone_locations)):
            xi = self.hydrophone_locations[i][0]
            yi = self.hydrophone_locations[i][1]
            zi = self.hydrophone_locations[i][2]
            di = np.sqrt((xi - x)**2 + (yi - x)**2 + (zi - x)**2)
            hydro_i_cost = (di - d0 - self.c * t[i])**2
            cost = cost + hydro_i_cost
        return cost


class ReceiverArraySim(object):
    """
        Simulates an array of receivers that listens to point sources and returns the DTOA.
        (difference in time of arrival)
        Base Units:
            time   - microseconds
            length - millimeters
    """
    def __init__(self, hydrophone_locations, wave_propagation_speed_mps):
        self.c = wave_propagation_speed_mps
        self.hydrophone_locations = np.array([0, 0, 0])
        for key in hydrophone_locations:
            sensor_location = np.array([hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
            self.hydrophone_locations = np.vstack((self.hydrophone_locations, sensor_location))
        self.hydrophone_locations = self.hydrophone_locations[1:]

    def listen(self, pulse):
        timestamps = []
        for idx in range(4):
            src_range = np.sqrt(sum(np.square(pulse.position() - self.hydrophone_locations[idx])))
            timestamps += [pulse.t + src_range / self.c]
        return np.array(timestamps)

class Pulse(object):
    """
    Represents an omnidirectional wave or impulse emmited from a point source
    """
    def __init__(self, x, y, z, t):
        self.x = x
        self.y = y
        self.z = z
        self.t = t

    def position(self):
        return np.array([self.x, self.y, self.z])

    def __repr__(self):
        return "Pulse:\t" + "x: " + str(self.x) + " y: " + str(self.y) + " z: " \
            + str(self.z) + " (mm)"


def quadratic(a, b, c):
    discriminant = b*b - 4*a*c
    if discriminant >= 0:
        first_times_a = (-b+math.copysign(math.sqrt(discriminant), -b))/2
        return [first_times_a/a, c/first_times_a]
    else:
        return []