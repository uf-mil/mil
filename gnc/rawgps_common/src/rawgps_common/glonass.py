from __future__ import division

import math

import numpy

L1_f0 = 1602e6

w_E = 0.729211510e-4

# to be correct, should take precession and nutation into account too
def inertial_from_ecef(t, (x, y, z)):
    th = w_E * t # needs offset
    return numpy.array([
        x*math.cos(th) - y*math.sin(th),
        x*math.sin(th) + y*math.cos(th),
        z,
    ])
def inertial_vel_from_ecef_vel(t, (x, y, z), inertial_point):
    th = w_E * t # needs offset
    return numpy.array([
        x*math.cos(th) - y*math.sin(th) - w_E * inertial_point[1],
        x*math.sin(th) + y*math.cos(th) + w_E * inertial_point[0],
        z,
    ])

def ecef_from_inertial(t, (x, y, z)):
    th = w_E * t # needs offset
    return numpy.array([
        +x*math.cos(th) + y*math.sin(th),
        -x*math.sin(th) + y*math.cos(th),
        z,
    ])
def ecef_vel_from_inertial_vel(t, (x, y, z), inertial_point):
    th = w_E * t # needs offset
    x += w_E * inertial_point[1]
    y -= w_E * inertial_point[0]
    return numpy.array([
        +x*math.cos(th) + y*math.sin(th),
        -x*math.sin(th) + y*math.cos(th),
        z,
    ])

def rk4_integrate(t0, Y0, F, t_end, stepsize):
    dt = t_end - t0
    steps = int(abs(dt / stepsize) + .5)
    if steps == 0:
        steps = 1
    #print 'steps:', steps
    h = dt / steps
    
    t = t0
    Y = Y0
    for i in xrange(steps):
        t = t0 + h * i
        K1 = F(t    , Y         )
        K2 = F(t+h/2, Y + h*K1/2)
        K3 = F(t+h/2, Y + h*K2/2)
        K4 = F(t+h  , Y + h*K3  )
        
        Y += h/6*(K1 + 2*K2 + 2*K3 + K4)
    return Y



class Ephemeris(object):
    def __init__(self, t_b, X, Y, Z, Vx, Vy, Vz, Ax, Ay, Az, gamma_n, tau_n):
        self.t_b, self.X, self.Y, self.Z, self.Vx, self.Vy, self.Vz, self.Ax, self.Ay, self.Az, self.gamma_n, self.tau_n = t_b, X, Y, Z, Vx, Vy, Vz, Ax, Ay, Az, gamma_n, tau_n
    
    def predict(self, t):
        day = 24*60*60
        while t > self.t_b + day/2: t -= day
        while t < self.t_b - day/2: t += day
        if not (abs(t - self.t_b) < 15*60):
            print 'ERROR: glonass ephemeris predicting too far'
        
        p0 = inertial_from_ecef(self.t_b, [self.X, self.Y, self.Z])
        v0 = inertial_vel_from_ecef_vel(self.t_b, [self.Vx, self.Vy, self.Vz], p0)
        a = inertial_from_ecef(self.t_b, [self.Ax, self.Ay, self.Az])
        
        a_E = 6378.136e3
        mu = 398600.44e9
        C_20 = -1082.63e-6
        
        def F(t, (x, y, z, vx, vy, vz)):
            pos = numpy.array([x, y, z])
            pos_bar = pos/numpy.linalg.norm(pos)
            mu_bar = mu / numpy.linalg.norm(pos)**2
            rho = a_E / numpy.linalg.norm(pos)
            acc = - mu_bar * pos_bar + 3/2 * C_20 * mu_bar * pos_bar * rho**2*numpy.array([1-5*pos_bar[2]**2, 1-5*pos_bar[2]**2, 3-5*pos_bar[2]**2]) + a
            return numpy.concatenate([numpy.array([vx, vy, vz]), acc])
        
        #print self.t_b, t
        Y = rk4_integrate(self.t_b, numpy.concatenate([p0, v0]), F, t, 30)
        
        #print 'Y:', Y
        
        return ecef_from_inertial(t, Y[:3]), ecef_vel_from_inertial_vel(t, Y[3:], Y[:3])
    
    def __str__(self):
        return 'Ephemeris(\n' + ''.join('    %s=%r\n' % (k, v) for k, v in sorted(self.__dict__.iteritems())) + ')'
