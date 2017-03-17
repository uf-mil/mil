from __future__ import division

import numpy
import scipy
import scipy.signal
import math

from hydrophones.msg import Ping

def resample(x, p, q):
    """Polyphase filter resample, based on MATLAB's resample.m"""
    bta = 5
    N = 10
    pqmax = max(p, q)
    fc = (1/2)/pqmax
    L = 2*N*pqmax + 1
    h = p*scipy.signal.firwin(L-1, 2*fc, window=('kaiser', bta))
    Lhalf = (L-1)/2
    Lx = len(x)

    nz = math.floor(q-(Lhalf % q))
    z = numpy.zeros(nz)
    Lhalf += nz

    delay = math.floor(math.ceil(Lhalf)/q)
    nz1 = 0
    while math.ceil(((Lx - 1)*p + len(h) + nz1)/q) - delay < math.ceil(Lx*p/q):
        nz1 = nz1+1;
    h = numpy.hstack([h, numpy.zeros(nz1)]);
    y = upfirdn(x,h,p,q)
    Ly = math.ceil(Lx*p/q)
    y = y[delay:]
    y = y[:Ly]
    return y

def upfirdn(x, h, p, q):
    # Allocate an expanded array to hold x interspersed with p-1 zeros,
    # padded with enough zeros for the fir filter
    x_expanded = numpy.zeros((x.shape[0] - 1)*p + h.shape[0])

    # Insert x values every p elements
    x_expanded[:x.shape[0]*p:p] = x

    # Run filter
    x_filt = scipy.signal.lfilter(h, 1, x_expanded)
    return x_filt

def make_ping_channel(delay=0, 
                      freq=25e3,
                      offset=0x7FFF,
                      ampl=200, 
                      zeros=64, 
                      count=1024,
                      sample_rate=300e3):
    w = 2*math.pi*freq/sample_rate
    sinwave = ampl*numpy.sin(w*(numpy.arange(count)-delay))

    delay_i = round(delay)
    window = numpy.zeros(count)
    window[zeros+delay_i:] = numpy.minimum(numpy.exp(numpy.arange(count - zeros - delay_i)/10), 2)-1

    noise = numpy.random.normal(0, .01, count)

    return offset + sinwave * window + noise

def make_ping(delays=[0, 0, 0, 0], args={}):
    return numpy.vstack(make_ping_channel(delay=delay, **args) for delay in delays)

def samples_to_list(samples):
    return samples.transpose().flatten().tolist()

def ping_to_samples(ping):
    return numpy.array(ping.data, dtype=numpy.float64).reshape((ping.samples, ping.channels)).transpose()
