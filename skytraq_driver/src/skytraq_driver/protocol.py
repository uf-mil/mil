from __future__ import division

import struct
import math

import numpy

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, PointStamped, Point

from rawgps_common import gps, bitstream
from rawgps_common.msg import Measurements, Satellite
from skytraq_driver.msg import Packet, PacketSet


class Handlers(object):
    def __init__(self, frame_id, gps_pub, gps_pos_pub):
        self.gps_pub = gps.GPSPublisher(frame_id, gps_pub, gps_pos_pub)
        self.frame_id = frame_id
        
        self.last_meas_time = None
    
    def gps_ephemeris_data(self, data, stamp):
        # just ignore this, subframes are received fast enough
        pass
    
    def ack(self, data, stamp):
        pass
    
    def sv_ch_status(self, data, stamp):
        pass
    
    def meas_time(self, data, stamp):
        IOD, WN, TOW_ms, measurement_period_ms = struct.unpack('>BHIH', data)
        self.last_meas_time = IOD, gps.Time(WN, TOW_ms*1e-3)
    
    def raw_meas(self, data, stamp):
        iod, nmeas = struct.unpack('>BB', data[0:2])
        assert len(data) == 2+nmeas*23
        measurements = [struct.unpack('>BBddfB', data[2+i*23:2+i*23+23])
            for i in xrange(nmeas)]
        
        if self.last_meas_time is None or self.last_meas_time[0] != iod:
            print "IOD didn't match!"
            return
        t = self.last_meas_time[1]
        
        sats = []
        for prn, cn0, pseudorange, acc_carrier_cycle, doppler_freq, status in measurements:
            (pseudorange_avail, doppler_freq_avail, acc_carrier_cycle_avail,
                cycle_slip_possible, integration_time_greater_10ms) = [
                bool(status & 2**i) for i in xrange(5)]
            if not pseudorange_avail or pseudorange == 0:
                pseudorange = None
            if not doppler_freq_avail:
                doppler_freq = None
            if not acc_carrier_cycle_avail or cycle_slip_possible:
                acc_carrier_cycle = None
            
            sats.append(dict(
                prn=prn,
                cn0=cn0,
                pseudo_range=pseudorange,
                carrier_cycles=acc_carrier_cycle,
                doppler_freq=doppler_freq,
            ))
        self.gps_pub.handle_raw_measurements(stamp, t, sats, math.floor(t.TOW*100+.5))
    
    def rcv_state(self, data, stamp):
        iod, navigation_state, wn, tow, ecef_x, ecef_y, ecef_z, ecef_dx, ecef_dy, ecef_dz, clock_bias, clock_drift, gdop, pdop, hdop, vdop, tdop = struct.unpack('>BBHddddfffdffffff', data)
        #if navigation_state >= 0x02: # require at least a 2D fix
        #    self.gps_pub.handle_pos_estimate(stamp, numpy.array([ecef_x, ecef_y, ecef_z]))
    
    def subframe(self, data, stamp):
        PRN, SFID, subframe_data = struct.unpack('>BB30s', data)
        self.gps_pub.handle_subframe(stamp, PRN, subframe_data)

message_ids = dict(
    configure_output_message_format=0x09,
    configure_binary_measurement_output_rates=0x12,
    get_almanac=0x11,
    get_ephemeris=0x30,
    software_version=0x80,
    software_crc=0x81,
    ack=0x83,
    nack=0x84,
    gps_almanac_data=0x87,
    gps_ephemeris_data=0xB1,
    meas_time=0xDC,
    raw_meas=0xDD,
    sv_ch_status=0xDE,
    rcv_state=0xDF,
    subframe=0xE0,
)
message_names = dict((id_, name) for name, id_ in message_ids.iteritems())

class SendProxy(object):
    def __init__(self, send_func):
        self._send_func = send_func
    
    def __getattr__(self, name):
        return lambda data: self._send_func(chr(message_ids[name]) + data)

def dispatch(message_id, body, obj, **kwargs):
    if message_id not in message_names:
        print hex(message_id), 'unknown'
        return
    message_name = message_names[message_id]
    if not hasattr(obj, message_name):
        print message_name, 'unhandled'
        return
    getattr(obj, message_name)(body, **kwargs)
