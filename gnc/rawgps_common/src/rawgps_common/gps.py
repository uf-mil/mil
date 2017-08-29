from __future__ import division

import math
from math import sin, cos, atan, sqrt, acos, pi, atan2
import os
import functools

import numpy
import yaml

from tf import transformations
from geometry_msgs.msg import Point, Vector3, PointStamped
from std_msgs.msg import Header

from rawgps_common.msg import Satellite, Measurements
import bitstream

xyz_array = lambda o: numpy.array([o.x, o.y, o.z])

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

# for i in xrange(100):
#     import random
#     x, y, z = random.uniform(-a, a), random.uniform(-a, a), random.uniform(-a, a)
#     lat, lon, height = latlongheight_from_ecef((x, y, z))
#     assert numpy.linalg.norm(ecef_from_latlongheight(lat, lon, height) - (x, y, z)) < 1e-6

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

class TLM(object):
    def __init__(self, data):
        assert len(data) == 3

        bs = bitstream.BitStream(data)
        preamble = bs.read(8)
        assert preamble == 0b10001011, bin(preamble)
        self.TLM_message = bs.read(14)
        self.integrity_status_flag = bs.read(1)
        bs.read(1) # reserved
        assert bs.at_end()

class HOW(object):
    def __init__(self, data):
        assert len(data) == 3

        bs = bitstream.BitStream(data)
        self.TOW = bs.read(17) * 6
        self.alert_flag = bs.read(1)
        self.antispoof_flag = bs.read(1)
        self.subframe_ID = bs.read(3)
        bs.read(2) # t
        assert bs.at_end()

class Subframe1(object):
    def __init__(self, data):
        assert len(data) == 30

        self.TLM = TLM(data[:3])

        self.HOW = HOW(data[3:6])
        assert self.HOW.subframe_ID == 1

        bs = bitstream.BitStream(data[6:])
        self.WN = bs.read(10)
        self.CA_or_P_on_L2 = bs.read(2)
        self.URA_index = bs.read(4)
        self.SV_health = bs.read(6)
        IODC_MSB = bs.read(2)
        self.L2_P_data_flag = bs.read(1)
        bs.read(23) # reserved
        bs.read(24) # reserved
        bs.read(24) # reserved
        bs.read(16) # reserved
        self.T_GD = bs.read_signed(8) * 2**-31
        self.IODC = IODC_MSB * 2**8 + bs.read(8)
        self.t_oc_TOW = bs.read(16) * 2**4
        self.a_f2 = bs.read_signed(8) * 2 **-55
        self.a_f1 = bs.read_signed(16) * 2**-43
        self.a_f0 = bs.read_signed(22) * 2**-31
        bs.read(2) # t
        assert bs.at_end()

        self.approx_recv_time = Time(self.WN, self.HOW.TOW)
        self.IODE = self.IODC % (2**8)

class Subframe2(object):
    def __init__(self, data):
        assert len(data) == 30

        self.TLM = TLM(data[:3])

        self.HOW = HOW(data[3:6])
        assert self.HOW.subframe_ID == 2

        bs = bitstream.BitStream(data[6:])
        self.IODE = bs.read(8)
        self.C_rs = bs.read_signed(16) * 2**-5
        self.Deltan = bs.read_signed(16) * 2**-43 * pi
        self.M_0 = bs.read_signed(32) * 2**-31 * pi
        self.C_uc = bs.read_signed(16) * 2**-29
        self.e = bs.read(32) * 2**-33
        self.C_us = bs.read_signed(16) * 2**-29
        self.sqrtA = bs.read(32) * 2**-19
        self.t_oe_TOW = bs.read(16) * 2**4
        self.fit_interval_flag = bs.read(1)
        self.AODO = bs.read(5) * 900
        bs.read(2) # t
        assert bs.at_end()

class Subframe3(object):
    def __init__(self, data):
        assert len(data) == 30

        self.TLM = TLM(data[:3])

        self.HOW = HOW(data[3:6])
        assert self.HOW.subframe_ID == 3

        bs = bitstream.BitStream(data[6:])
        self.C_ic = bs.read_signed(16) * 2**-29
        self.Omega_0 = bs.read_signed(32) * 2**-31 * pi
        self.C_is = bs.read_signed(16) * 2**-29
        self.i_0 = bs.read_signed(32) * 2**-31 * pi
        self.C_rc = bs.read_signed(16) * 2**-5
        self.omega = bs.read_signed(32) * 2**-31 * pi
        self.Omega_dot = bs.read_signed(24) * 2**-43 * pi
        self.IODE = bs.read(8)
        self.IDOT = bs.read_signed(14) * 2**-43 * pi
        bs.read(2) # t
        assert bs.at_end()

class Subframe4(object):
    def __init__(self, data):
        assert len(data) == 30

        self.TLM = TLM(data[:3])

        self.HOW = HOW(data[3:6])
        assert self.HOW.subframe_ID == 4

        bs = bitstream.BitStream(data[6:])
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
        else:
            # ...
            pass

class Subframe5(object):
    def __init__(self, data):
        assert len(data) == 30

        self.TLM = TLM(data[:3])

        self.HOW = HOW(data[3:6])
        assert self.HOW.subframe_ID == 5

        bs = bitstream.BitStream(data[6:])
        self.data_id = bs.read(2)
        self.sv_id = bs.read(6)
        # ...

subframes = {
    1: Subframe1, 2: Subframe2, 3: Subframe3,
    4: Subframe4, 5: Subframe5,
}
def parse_subframe(data):
    assert len(data) == 30
    TLM(data[:3])
    return subframes[HOW(data[3:6]).subframe_ID](data)

class Ephemeris(object):
    def __init__(self, subframe_1, subframe_2, subframe_3):
        assert subframe_1.IODE == subframe_2.IODE == subframe_3.IODE
        self.__dict__.update(subframe_1.__dict__)
        self.__dict__.update(subframe_2.__dict__)
        self.__dict__.update(subframe_3.__dict__)

        self.t_oc = self.approx_recv_time.new_minimizing_dt(self.t_oc_TOW)
        self.t_oe = self.approx_recv_time.new_minimizing_dt(self.t_oe_TOW)

    def predict_pos_deltat_r(self, t):
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
        pos, deltat_r = self.predict_pos_deltat_r(t)
        vel = (self.predict_pos_deltat_r(t+.1)[0] - self.predict_pos_deltat_r(t-.1)[0])/.2
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

class GPSPublisher(object):
    def __init__(self, frame_id, pub, pos_pub):
        self.frame_id = frame_id

        self.ephemeris_data = {} # prn -> (iode -> [frame*3])
        self.ephemerises = {}

        home = os.path.expanduser('~')
        if home == '~': raise AssertionError("home path expansion didn't work")
        self._ionospheric_model_path = os.path.join(home, '.ros',
            'rawgps_common', 'ionospheric_model.yaml')
        if os.path.exists(self._ionospheric_model_path):
            with open(self._ionospheric_model_path, 'rb') as f:
                self.ionospheric_model = IonosphericModel(**yaml.load(f))
        else:
            self.ionospheric_model = None

        self.pub = pub
        self.pos_pub = pos_pub

        self._last_pos = None

    def _set_ionospheric_model(self, ionospheric_model):
        self.ionospheric_model = ionospheric_model

        if not os.path.exists(os.path.dirname(self._ionospheric_model_path)):
            os.makedirs(os.path.dirname(self._ionospheric_model_path))
        with open(self._ionospheric_model_path, 'wb') as f:
            yaml.dump(dict(
                a=self.ionospheric_model.a,
                b=self.ionospheric_model.b,
            ), f)

    def handle_raw_measurements(self, stamp, gps_time, sats, sync=nan):
        satellites = []
        for sat in sats:
            #print sat['prn'], sat['cn0'], sat['pseudo_range'], sat['carrier_cycles'], sat['doppler_freq']

            if sat['prn'] not in self.ephemerises:
                #print 'no ephemeris, dropping', sat['prn']
                continue
            eph = self.ephemerises[sat['prn']]
            if not eph.is_healthy():
                #print 'unhealthy, dropping', sat['prn']
                continue

            sat_msg = generate_satellite_message(
                sat['prn'], eph, sat['cn0'], gps_time,
                sat['pseudo_range'], sat['carrier_cycles'], sat['doppler_freq'],
                self._last_pos, self.ionospheric_model)
            if sat_msg is not None:
                satellites.append(sat_msg)

        if len(satellites) >= 4:
            pos_estimate = estimate_pos(satellites, use_corrections=self._last_pos is not None)
            self.pos_pub.publish(PointStamped(
                header=Header(
                    stamp=stamp,
                    frame_id='/ecef',
                ),
                point=Point(*pos_estimate),
            ))
        else:
            pos_estimate = None
        self._last_pos = pos_estimate

        satellites = []
        for sat in sats:
            print sat['prn'], sat['cn0'], sat['pseudo_range'], sat['carrier_cycles'], sat['doppler_freq']

            if sat['prn'] not in self.ephemerises:
                print 'no ephemeris, dropping', sat['prn']
                continue
            eph = self.ephemerises[sat['prn']]
            if not eph.is_healthy():
                print 'unhealthy, dropping', sat['prn']
                continue

            sat_msg = generate_satellite_message(
                sat['prn'], eph, sat['cn0'], gps_time,
                sat['pseudo_range'], sat['carrier_cycles'], sat['doppler_freq'],
                pos_estimate, self.ionospheric_model)
            if sat_msg is not None:
                satellites.append(sat_msg)

        self.pub.publish(Measurements(
            header=Header(
                stamp=stamp,
                frame_id=self.frame_id,
            ),
            sync_WN=gps_time.WN,
            sync=gps_time.TOW,
            position=Point(*pos_estimate if pos_estimate is not None else (0, 0, 0)),
            position_valid=pos_estimate is not None,
            satellites=satellites,
        ))
        print

    def handle_subframe(self, stamp, prn, data):
        assert len(data) == 30
        subframe = parse_subframe(data)

        if subframe.HOW.subframe_ID in [1, 2, 3]:
            self.ephemeris_data.setdefault(prn, {}).setdefault(subframe.IODE, [None]*3)[subframe.HOW.subframe_ID-1] = subframe
            self._ephemeris_think(prn, subframe.IODE)
        elif subframe.HOW.subframe_ID == 4:
            if subframe.sv_id == 56: # page 18
                self._set_ionospheric_model(IonosphericModel(subframe.alpha, subframe.beta))

    def _ephemeris_think(self, prn, iode):
        subframes = self.ephemeris_data[prn][iode]
        if any(x is None for x in subframes): return
        self.ephemerises[prn] = Ephemeris(*subframes)

def generate_satellite_message(prn, eph, cn0, gps_t, pseudo_range, carrier_cycles, doppler_freq, approximate_receiver_position=None, ionospheric_model=None):
    if pseudo_range is None: # need pseudorange
        return None

    t_SV = gps_t - pseudo_range/c

    t = t_SV # initialize
    deltat_r = 0
    for i in xrange(2):
        dt = t - eph.t_oc; assert abs(dt) < week_length/2
        deltat_SV_L1 = eph.a_f0 + eph.a_f1 * dt + eph.a_f2 * dt**2 + deltat_r - eph.T_GD
        t = t_SV - deltat_SV_L1
        if i == 1:
            sat_pos, deltat_r, sat_vel = eph.predict(t)
        else:
            sat_pos, deltat_r = eph.predict_pos_deltat_r(t)

    if approximate_receiver_position is None:
        direction = direction_enu = numpy.array([nan, nan, nan])
        T_iono = nan
        T_tropo = nan
    else:
        direction = sat_pos - approximate_receiver_position; direction /= numpy.linalg.norm(direction)
        direction_enu = enu_from_ecef(direction, approximate_receiver_position)
        T_iono = ionospheric_model.evaluate(approximate_receiver_position,
            sat_pos, gps_t) if ionospheric_model is not None else nan
        T_tropo = tropospheric_model(approximate_receiver_position, sat_pos)/c


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
        T_iono=T_iono,
        T_tropo=T_tropo,
        carrier_distance=carrier_cycles*c/L1_f0 if carrier_cycles is not None else nan,
        doppler_velocity=doppler_freq*c/L1_f0,

        direction_enu=Vector3(*direction_enu),
        velocity_plus_drift=doppler_freq*c/L1_f0 + direction.dot(sat_vel) if doppler_freq is not None else nan,
    )



def estimate_pos(sats, use_corrections, quiet=False, pos_guess=None):
    if pos_guess is None:
        pos_guess = [0, 0, 0]

    def mean(xs):
        xs = list(xs)
        return sum(xs)/len(xs)

    def find_minimum(x0, residuals):
        #print 'x0', x0
        x = x0
        for i in xrange(6):
            r, J = residuals(x)
            #print 'r', r
            #print sum(r)
            if not quiet: print '|r|', numpy.linalg.norm(r)/math.sqrt(len(r)), use_corrections, x
            #print 'J', J
            try:
                # dx = (J^T J)^-1 J^T r
                # (J^T J) dx = J^T r
                # dx = numpy.linalg.solve(J^T J, J^T r)
                x = x - numpy.linalg.solve(J.T.dot(J), J.T.dot(r))
            except:
                numpy.set_printoptions(threshold=numpy.nan)
                print repr(sats), use_corrections
                print 'J =', J
                raise
            #lat, lon, height = latlongheight_from_ecef(x[:3])
            #print 'x', x, '=', math.degrees(lat), math.degrees(lon), height
        return x

    def residuals(x):
        assert len(x) == 4
        pos = x[0:3]
        t = x[3]/c

        import glonass # for inertial_from_ecef
        #for sat in sats:
        #    print sat.prn, (sat.T_iono + sat.T_tropo if use_corrections else 0)*c
        return [
            numpy.linalg.norm(
                glonass.inertial_from_ecef(t, pos) -
                glonass.inertial_from_ecef(sat.time, xyz_array(sat.position))
            ) - (t - sat.time - (sat.T_iono + sat.T_tropo if use_corrections else 0))*c
        for sat in sats], numpy.array([[transformations.unit_vector(
                glonass.inertial_from_ecef(t, pos) -
                glonass.inertial_from_ecef(sat.time, xyz_array(sat.position))
            ).dot(glonass.inertial_from_ecef(t, [1, 0, 0])), transformations.unit_vector(
                glonass.inertial_from_ecef(t, pos) -
                glonass.inertial_from_ecef(sat.time, xyz_array(sat.position))
            ).dot(glonass.inertial_from_ecef(t, [0, 1, 0])), transformations.unit_vector(
                glonass.inertial_from_ecef(t, pos) -
                glonass.inertial_from_ecef(sat.time, xyz_array(sat.position))
            ).dot(glonass.inertial_from_ecef(t, [0, 0, 1])), transformations.unit_vector(
                glonass.inertial_from_ecef(t, pos) -
                glonass.inertial_from_ecef(sat.time, xyz_array(sat.position))
            ).dot(glonass.inertial_vel_from_ecef_vel(t, [0, 0, 0], pos))/c-1] for sat in sats])
    x = find_minimum([pos_guess[0], pos_guess[1], pos_guess[2],
        mean((sat.time+(sat.T_iono+sat.T_tropo if use_corrections else 0))*c + numpy.linalg.norm(xyz_array(sat.position) - pos_guess) for sat in sats),
        #0,
    ], residuals)
    pos = x[:3]
    t = x[3]/c

    return pos

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


