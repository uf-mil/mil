from __future__ import division

import math
from math import sin, cos, atan, sqrt, acos, pi, atan2
import functools

import numpy

import roslib
roslib.load_manifest('tf')
from tf import transformations
from geometry_msgs.msg import Point, Vector3

from rawgps_common.msg import Satellite
import bitstream

deriv = lambda f, x: (f(x+.1)-f(x-.1))/.2

inf = 1e1000; assert math.isinf(inf)
nan = inf/inf

# WGS 84
a = 6378137.0
f = 1/298.257223563

e = sqrt(2*f - f**2)
b = a*(1 - f)

def ecef_from_latlongheight(latitude, longitude, height):

    N = a/sqrt(1 - e**2*sin(latitude)**2)
    return numpy.array([
        (N + height)*cos(latitude)*cos(longitude),
        (N + height)*cos(latitude)*sin(longitude),
        (N*(1 - e**2) + height)*sin(latitude),
    ])
def latlongheight_from_ecef((x, y, z)):
    # Ferrari's solution
    zeta = (1 - e**2) * z**2 / a**2
    p = math.sqrt(x**2 + y**2)
    rho = (p**2 / a**2 + zeta - e**4) / 6
    s = e**4 * zeta * p**2 / (4 * a**2)
    t = (rho**3 + s + math.sqrt(s * (s + 2*rho**3)))**(1/3)
    u = rho + t + rho**2 / t
    v = math.sqrt(u**2 + e**4 * zeta)
    w = e**2 * (u + v - zeta) / (2 * v)
    k = 1 + e**2*(math.sqrt(u + v + w**2) + w) / (u + v)
    k0 = (1 - e**2)**-1
    h = e**-2*(k**-1-k0**-1)*math.sqrt(p**2 + z**2*k**2)
    return math.atan(z*k/p), math.atan2(y, x), h

for i in xrange(100):
    import random
    x, y, z = random.uniform(-a, a), random.uniform(-a, a), random.uniform(-a, a)
    lat, lon, height = latlongheight_from_ecef((x, y, z))
    assert numpy.linalg.norm(ecef_from_latlongheight(lat, lon, height) - (x, y, z)) < 1e-6

def enu_from_ecef_tf(ecef_pos):
    up_ecef = transformations.unit_vector(ecef_from_latlongheight(*latlongheight_from_ecef(ecef_pos)+numpy.array([0, 0, 1]))-ecef_pos)
    east_ecef = transformations.unit_vector(numpy.cross([0, 0, 1], up_ecef))
    north_ecef = numpy.cross(up_ecef, east_ecef)
    enu_from_ecef = numpy.array([east_ecef, north_ecef, up_ecef])
    return enu_from_ecef

def enu_from_ecef(ecef_v, ecef_pos):
    return enu_from_ecef_tf(ecef_pos).dot(ecef_v)
def ecef_from_enu(enu_v, ecef_pos):
    return enu_from_ecef_tf(ecef_pos).T.dot(enu_v)

def newton(x0, f, f_prime):
    x = x0
    for i in xrange(10):
        x -= f(x)/f_prime(x)
    assert abs(f(x)/f_prime(x)/x) < 1e-6
    return x

week_length = 7*24*60*60
@functools.total_ordering
class Time(object):
    def __init__(self, WN, TOW):
        self.WN = WN
        self.TOW = TOW
        while self.TOW >= week_length:
            self.WN += 1
            self.TOW -= week_length
        while self.TOW < 0:
            self.WN -= 1
            self.TOW += week_length
        self.WN = self.WN % 1024
    def __repr__(self):
        return 'gps.Time(%r, %r)' % (self.WN, self.TOW)
    def __sub__(self, other):
        if not isinstance(other, Time):
            return self + (-other)
        res = week_length*(self.WN - other.WN) + (self.TOW - other.TOW)
        while res > 512*week_length:
            res -= 1024*week_length
        while res < -512*week_length:
            res += 1024*week_length
        return res
    def __add__(self, other):
        assert not isinstance(other, Time)
        return Time(self.WN, self.TOW + other)
    def __eq__(self, other):
        return self.WN == other.WN and self.TOW == other.TOW
    def __ne__(self, other):
        return not (self == other)
    def __lt__(self, other):
        assert isinstance(other, Time)
        return self - other < 0
    def new_minimizing_dt(self, TOW):
        res = Time(self.WN, TOW)
        
        while abs(Time(res.WN-1, res.TOW) - self) < abs(res - self):
            res = Time(res.WN-1, res.TOW)
        while abs(Time(res.WN+1, res.TOW) - self) < abs(res - self):
            res = Time(res.WN+1, res.TOW)
        
        return res

L1_f0 = 1575.42e6 # Hz
c = 299792458 # m/s
mu = 3.986005e14 # m^3/s^2
omega_dot_e = 7.2921151467e-5 # rad/s

class SubframeHeader(object):
    def __init__(self, data):
        assert len(data) == 3
        bs = bitstream.BitStream(data)
        self.TLM = bs.read(22)
        self.TLM_C = bs.read(2)
        assert data.at_end()

class HOW(object):
    def __init__(self, bs):
        self.TOW = bs.read(17) * 6
        self.alert_flag = bs.read(1)
        self.antispoof_flag = bs.read(1)
        self.subframe_ID = bs.read(3)
        bs.read(2) # t

class Subframe1(object):
    def __init__(self, data):
        assert len(data) == 27
        subframe_1 = bitstream.BitStream(data)
        
        self.HOW = HOW(subframe_1)
        assert self.HOW.subframe_ID == 1
        
        self.WN = subframe_1.read(10)
        self.CA_or_P_on_L2 = subframe_1.read(2)
        self.URA_index = subframe_1.read(4)
        self.SV_health = subframe_1.read(6)
        IODC_MSB = subframe_1.read(2)
        self.L2_P_data_flag = subframe_1.read(1)
        subframe_1.read(23) # reserved
        subframe_1.read(24) # reserved
        subframe_1.read(24) # reserved
        subframe_1.read(16) # reserved
        self.T_GD = subframe_1.read_signed(8) * 2**-31
        self.IODC = IODC_MSB * 2**8 + subframe_1.read(8)
        self.t_oc_TOW = subframe_1.read(16) * 2**4
        self.a_f2 = subframe_1.read_signed(8) * 2 **-55
        self.a_f1 = subframe_1.read_signed(16) * 2**-43
        self.a_f0 = subframe_1.read_signed(22) * 2**-31
        subframe_1.read(2) # t
        assert subframe_1.at_end()
        
        self.approx_recv_time = Time(self.WN, self.HOW.TOW)
        self.IODE = self.IODC % (2**8)

class Subframe2(object):
    def __init__(self, data):
        assert len(data) == 27
        subframe_2 = bitstream.BitStream(data)
        
        self.HOW = HOW(subframe_2)
        assert self.HOW.subframe_ID == 2
        
        self.IODE = subframe_2.read(8)
        self.C_rs = subframe_2.read_signed(16) * 2**-5
        self.Deltan = subframe_2.read_signed(16) * 2**-43 * pi
        self.M_0 = subframe_2.read_signed(32) * 2**-31 * pi
        self.C_uc = subframe_2.read_signed(16) * 2**-29
        self.e = subframe_2.read(32) * 2**-33
        self.C_us = subframe_2.read_signed(16) * 2**-29
        self.sqrtA = subframe_2.read(32) * 2**-19
        self.t_oe_TOW = subframe_2.read(16) * 2**4
        self.fit_interval_flag = subframe_2.read(1)
        self.AODO = subframe_2.read(5) * 900
        subframe_2.read(2) # t
        assert subframe_2.at_end()

class Subframe3(object):
    def __init__(self, data):
        assert len(data) == 27
        subframe_3 = bitstream.BitStream(data)
        
        self.HOW = HOW(subframe_3)
        assert self.HOW.subframe_ID == 3
        
        self.C_ic = subframe_3.read_signed(16) * 2**-29
        self.Omega_0 = subframe_3.read_signed(32) * 2**-31 * pi
        self.C_is = subframe_3.read_signed(16) * 2**-29
        self.i_0 = subframe_3.read_signed(32) * 2**-31 * pi
        self.C_rc = subframe_3.read_signed(16) * 2**-5
        self.omega = subframe_3.read_signed(32) * 2**-31 * pi
        self.Omega_dot = subframe_3.read_signed(24) * 2**-43 * pi
        self.IODE = subframe_3.read(8)
        self.IDOT = subframe_3.read_signed(14) * 2**-43 * pi
        subframe_3.read(2) # t
        assert subframe_3.at_end()

class Subframe4(object):
    def __init__(self, data):
        assert len(data) == 27
        bs = bitstream.BitStream(data)
        
        self.HOW = HOW(bs)
        assert self.HOW.subframe_ID == 4
        
        self.data_id = bs.read(2)
        self.sv_id = bs.read(6)
        
        if self.sv_id == 56: # page 18
            self.alpha = [
                bs.read_signed(8) * 2**-30,
                bs.read_signed(8) * 2**-27,
                bs.read_signed(8) * 2**-24,
                bs.read_signed(8) * 2**-24,
            ]
            self.beta = [
                bs.read_signed(8) * 2**11,
                bs.read_signed(8) * 2**14,
                bs.read_signed(8) * 2**16,
                bs.read_signed(8) * 2**16,
            ]
            self.A_1 = bs.read_signed(24) * 2**-50
            self.A_0 = bs.read_signed(32) * 2**-30
            self.t_ot = bs.read(8) * 2**12
            self.WN_t = bs.read(8)
            self.Deltat_LS = bs.read_signed(8)
            self.WN_LSF = bs.read(8)
            self.DN = bs.read(8)
            self.Deltat_LSF = bs.read_signed(8)
            bs.read(14) # reserved
            bs.read(2) # t
            assert bs.at_end()

class Subframe5(object):
    def __init__(self, data):
        assert len(data) == 27
        bs = bitstream.BitStream(data)
        
        self.HOW = HOW(bs)
        assert self.HOW.subframe_ID == 5
        
        self.data_id = bs.read(2)
        self.sv_id = bs.read(6)

subframes = {
    1: Subframe1, 2: Subframe2, 3: Subframe3,
    4: Subframe4, 5: Subframe5,
}

class Ephemeris(object):
    def __init__(self, subframe_1, subframe_2, subframe_3):
        assert subframe_1.IODE == subframe_2.IODE == subframe_3.IODE
        self.__dict__.update(subframe_1.__dict__)
        self.__dict__.update(subframe_2.__dict__)
        self.__dict__.update(subframe_3.__dict__)
        
        self.t_oc = self.approx_recv_time.new_minimizing_dt(self.t_oc_TOW)
        self.t_oe = self.approx_recv_time.new_minimizing_dt(self.t_oe_TOW)
    
    def _predict(self, t):
        A = self.sqrtA**2
        n_0 = sqrt(mu/A**3)
        t_k = t - self.t_oe
        assert abs(t_k) < week_length/2
        if not (abs(t_k) < 6*60*60):
            print 'ERROR: ephemeris predicting more than 6 hours from now (%f hours)' % (t_k/60/60,)
        n = n_0 + self.Deltan
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
        
        omega_k = self.Omega_0 + (self.Omega_dot - omega_dot_e) * t_k - omega_dot_e * self.t_oe.TOW
        
        x_k = x_k_prime * cos(omega_k) - y_k_prime * cos(i_k) * sin(omega_k)
        y_k = x_k_prime * sin(omega_k) + y_k_prime * cos(i_k) * cos(omega_k)
        z_k = y_k_prime * sin(i_k)
        
        F = -2*mu**(1/2)/c**2
        deltat_r = F * self.e * A**(1/2) * sin(E_k)
        
        return numpy.array([x_k, y_k, z_k]), deltat_r
    
    def predict(self, t):
        pos, deltat_r = self._predict(t)
        vel = (self._predict(t+.1)[0] - self._predict(t-.1)[0])/.2
        return pos, deltat_r, vel
    
    def is_healthy(self):
        return self.SV_health == 0
    
    def __str__(self):
        return 'Ephemeris(\n' + ''.join('    %s=%r\n' % (k, v) for k, v in sorted(self.__dict__.iteritems())) + ')'

class IonosphericModel(object):
    def __init__(self, a, b):
        assert len(a) == 4 and len(b) == 4
        self.a, self.b = a, b
    
    def evaluate(self, ground_pos_ecef, sat_pos_ecef, GPS_time):
        lat, lon, height = latlongheight_from_ecef(ground_pos_ecef)
        sat_pos_enu = enu_from_ecef(sat_pos_ecef - ground_pos_ecef, ground_pos_ecef)
        sat_dir_enu = sat_pos_enu / numpy.linalg.norm(sat_pos_enu)
        
        E = math.asin(sat_dir_enu[2]) / math.pi
        A = math.atan2(sat_dir_enu[0], sat_dir_enu[1]) / math.pi
        phi_u = lat / math.pi
        lambda_u = lon / math.pi
        
        psi = 0.0137/(E + 0.11) - 0.022
        
        phi_i = phi_u + psi * math.cos(A * math.pi)
        if phi_i > 0.416: phi_i = 0.416
        if phi_i < -0.416: phi_i = -0.416
        
        lambda_i = lambda_u + psi * math.sin(A * math.pi) / math.cos(phi_i * math.pi)
        phi_m = phi_i + 0.064 * math.cos((lambda_i - 1.617) * math.pi)
        
        t = (4.32e4 * lambda_i + GPS_time.TOW) % 86400
        
        F = 1 + 16 * (0.53 - E)**3
        
        PER = sum(self.b[n] * phi_m**n for n in xrange(4))
        if PER < 72000: PER = 72000
        
        AMP = sum(self.a[n] * phi_m**n for n in xrange(4))
        if AMP < 0: AMP = 0
        
        x = 2*math.pi*(t - 50400)/PER
        
        if abs(x) < 1.57:
            T_iono = F * (5e-9 + AMP * (1 - x**2/2 + x**4/24))
        else:
            T_iono = F * 5e-9
        
        return T_iono

def tropospheric_model(ground_pos_ecef, sat_pos_ecef): # returns meters
    sat_pos_enu = enu_from_ecef(sat_pos_ecef - ground_pos_ecef, ground_pos_ecef)
    sat_dir_enu = sat_pos_enu / numpy.linalg.norm(sat_pos_enu)
    
    E = math.asin(sat_dir_enu[2])
    
    return 2.312 / math.sin(math.sqrt(E * E + 1.904E-3)) + \
           0.084 / math.sin(math.sqrt(E * E + 0.6854E-3))

def generate_satellite_message(prn, eph, cn0, gps_t, pseudo_range, carrier_cycles, doppler_freq, approximate_receiver_position=None, ionospheric_model=None):
    t_SV = gps_t - pseudo_range/c
    
    t = t_SV # initialize
    deltat_r = 0
    for i in xrange(3):
        dt = t - eph.t_oc
        assert abs(dt) < week_length/2
        deltat_SV_L1 = eph.a_f0 + eph.a_f1 * dt + eph.a_f2 * dt**2 + deltat_r - eph.T_GD
        t = t_SV - deltat_SV_L1
        sat_pos, deltat_r, sat_vel = eph.predict(t)
        #print i, -pseudo_range * 1e-3 - deltat_SV_L1
        #print i, sat_pos
    
    if approximate_receiver_position is not None:
        direction = sat_pos - approximate_receiver_position; direction /= numpy.linalg.norm(direction)
        direction_enu = enu_from_ecef(direction, approximate_receiver_position)
        velocity_plus_drift = doppler_freq*c/L1_f0 + direction.dot(sat_vel)
        if ionospheric_model is not None:
            T_iono = ionospheric_model.evaluate(approximate_receiver_position, sat_pos, t_SV) # technically should be receiver time, but doesn't matter
            print "iono", -T_iono * c
        else:
            #print 'no model'
            T_iono = 0
        D_tropo = tropospheric_model(approximate_receiver_position, sat_pos)
    else:
        return None
    
    
    sat_pos_enu = enu_from_ecef(sat_pos - approximate_receiver_position, approximate_receiver_position)
    sat_dir_enu = sat_pos_enu / numpy.linalg.norm(sat_pos_enu)
    
    E = math.asin(sat_dir_enu[2])
    if E < math.radians(5):
        return None
    
    return Satellite(
        prn=prn,
        cn0=cn0,
        position=Point(*sat_pos),
        velocity=Vector3(*sat_vel),
        time=-pseudo_range/c - deltat_SV_L1,
        T_iono=T_iono + D_tropo/c,
        carrier_distance=carrier_cycles*c/L1_f0,
        doppler_velocity=doppler_freq*c/L1_f0,
        
        direction_enu=Vector3(*direction_enu),
        velocity_plus_drift=velocity_plus_drift,
    )

if __name__ == '__main__':
    station_pos = ecef_from_latlongheight(math.radians(40), -math.radians(100), 0)
    print 'station_pos', station_pos
    sat_pos_enu = 5000e3 * numpy.array([
        math.cos(math.radians(20))*math.sin(math.radians(210)),
        math.cos(math.radians(20))*math.cos(math.radians(210)),
        math.sin(math.radians(20)),
    ])
    print 'sat_pos_enu', sat_pos_enu
    sat_pos = ecef_from_enu(sat_pos_enu, station_pos) + station_pos
    print 'sat_pos', sat_pos
    
    im = IonosphericModel(
        [3.82e-8, 1.49e-8, -1.79e-7, 0],
        [1.43e5, 0, -3.28e5, 1.13e5],
    )
    
    print
    
    print im.evaluate(station_pos, sat_pos, (20*60+45)*60)


