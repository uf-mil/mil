from math import sin, cos, atan, sqrt, acos, pi, atan2

import numpy

import roslib
roslib.load_manifest('tf')
from tf import transformations

import bitstream

deriv = lambda f, x: (f(x+.1)-f(x-.1))/.2

def ecef_from_latlongheight(latitude, longitude, height):
    # WGS 84
    a = 6378137.0
    f = 1/298.257223563

    e = sqrt(2*f - f**2)

    N = a/sqrt(1 - e**2*sin(latitude)**2)
    return numpy.array([
        (N + height)*cos(latitude)*cos(longitude),
        (N + height)*cos(latitude)*sin(longitude),
        (N*(1 - e**2) + height)*sin(latitude),
    ])


def enu_from_ecef(v, pos):
    up_ecef = transformations.unit_vector(pos)
    east_ecef = transformations.unit_vector(numpy.cross([0, 0, 1], up_ecef))
    north_ecef = numpy.cross(up_ecef, east_ecef)
    enu_from_ecef = numpy.array([east_ecef, north_ecef, up_ecef])
    return enu_from_ecef.dot(v)

def newton(x0, f, f_prime):
    x = x0
    for i in xrange(10):
        x -= f(x)/f_prime(x)
    assert abs(f(x)/f_prime(x)/x) < 1e-6
    return x


L1_f0 = 1575.42e6
c = 299792458
mu = 3.986005e14
omega_dot_e = 7.2921151467e-5

class Ephemeris(object):
    def __init__(self, subframe_1_data, subframe_2_data, subframe_3_data):
        subframe_1 = bitstream.BitStream(subframe_1_data)
        self.HOW = subframe_1.read(22)
        subframe_1.read(2) # t
        self.WN = subframe_1.read(10)
        subframe_1.read(2) # reserved
        self.URA_index = subframe_1.read(4)
        self.health = subframe_1.read(6)
        iodc_msb = subframe_1.read(2)
        subframe_1.read(1) # reserved
        subframe_1.read(23) # reserved
        subframe_1.read(24) # reserved
        subframe_1.read(24) # reserved
        subframe_1.read(16) # reserved
        self.T_GD = subframe_1.read_signed(8) * 2**-31
        self.iodc = iodc_msb*2**256  + subframe_1.read(8)
        self.t_oc = subframe_1.read(16) * 2**4
        self.a_f2 = subframe_1.read_signed(8) * 2 **-55
        self.a_f1 = subframe_1.read_signed(16) * 2**-43
        self.a_f0 = subframe_1.read_signed(22) * 2**-31
        subframe_1.read(2) # t
        assert subframe_1.at_end()
        
        subframe_2 = bitstream.BitStream(subframe_2_data)
        self.HOW2 = subframe_2.read(22)
        subframe_2.read(2)
        self.IODE = subframe_2.read(8)
        self.C_rs = subframe_2.read_signed(16) * 2**-5
        self.delta_n = subframe_2.read_signed(16) * 2**-43 * pi
        self.M_0 = subframe_2.read_signed(32) * 2**-31 * pi
        self.C_uc = subframe_2.read_signed(16) * 2**-29
        self.e = subframe_2.read(32) * 2**-33
        self.C_us = subframe_2.read_signed(16) * 2**-29
        self.sqrt_A = subframe_2.read(32) * 2**-19
        self.t_oe = subframe_2.read(16) * 2**4
        subframe_2.read(1)
        subframe_2.read(5)
        subframe_2.read(2)
        assert subframe_2.at_end()
        
        subframe_3 = bitstream.BitStream(subframe_3_data)
        self.HOW3 = subframe_3.read(22)
        subframe_3.read(2)
        self.C_ic = subframe_3.read_signed(16) * 2**-29
        self.omega_0 = subframe_3.read_signed(32) * 2**-31 * pi
        self.C_is = subframe_3.read_signed(16) * 2**-29
        self.i_0 = subframe_3.read_signed(32) * 2**-31 * pi
        self.C_rc = subframe_3.read_signed(16) * 2**-5
        self.omega = subframe_3.read_signed(32) * 2**-31 * pi
        self.omega_dot = subframe_3.read_signed(24) * 2**-43 * pi # mislabeled as C in spec?
        self.IODE2 = subframe_3.read(8)
        self.IDOT = subframe_3.read_signed(14) * 2**-43 * pi
        subframe_3.read(2)
        assert subframe_3.at_end()
        
        self.t_oe += self.WN * 7*24*60*60
    
    def _predict(self, t):
        A = self.sqrt_A**2
        n_0 = sqrt(mu/A**3)
        t_k = t - self.t_oe
        time_wraparound = 1024*7*24*60*60
        while t_k > +time_wraparound/2: t_k -= time_wraparound
        while t_k < -time_wraparound/2: t_k += time_wraparound
        if not (abs(t_k) < 6*60*60):
            print 'ERROR: ephemeris predicting more than 6 hours from now (%f hours)' % (t_k/60/60,)
        n = n_0 + self.delta_n
        M_k = self.M_0 + n * t_k
        E_k = newton(M_k,
            lambda E_k: E_k - self.e*sin(E_k) - M_k,
            lambda E_k: 1   - self.e*cos(E_k))
        #v_k = atan(sqrt(1-self.e**2)*sin(E_k)/(cos(E_k) - self.e))
        v_k = atan2(sqrt(1-self.e**2)*sin(E_k), (cos(E_k) - self.e))
        other_E_k = acos((self.e + cos(v_k))/(1 + self.e*cos(v_k)))
        phi_k = v_k + self.omega
        
        du_k = self.C_us * sin(2*phi_k) + self.C_uc * cos(2*phi_k)
        dr_k = self.C_rc * cos(2*phi_k) + self.C_rs * sin(2*phi_k)
        di_k = self.C_ic * cos(2*phi_k) + self.C_is * sin(2*phi_k)
        
        u_k = phi_k + du_k
        r_k = A*(1 - self.e*cos(E_k)) + dr_k
        i_k = self.i_0 + di_k + self.IDOT * t_k
        
        x_k_prime = r_k * cos(u_k)
        y_k_prime = r_k * sin(u_k)
        
        omega_k = self.omega_0 + (self.omega_dot - omega_dot_e) * t_k - omega_dot_e * self.t_oe
        
        x_k = x_k_prime * cos(omega_k) - y_k_prime * cos(i_k) * sin(omega_k)
        y_k = x_k_prime * sin(omega_k) + y_k_prime * cos(i_k) * cos(omega_k)
        z_k = y_k_prime * sin(i_k)
        
        return numpy.array([x_k, y_k, z_k])
    
    def predict(self, t):
        return self._predict(t), (self._predict(t+.1)-self._predict(t-.1))/.2
    
    def is_healthy(self):
        return self.health == 0
    
    def __str__(self):
        return 'Ephemeris(\n' + ''.join('    %s=%r\n' % (k, v) for k, v in sorted(self.__dict__.iteritems())) + ')'

