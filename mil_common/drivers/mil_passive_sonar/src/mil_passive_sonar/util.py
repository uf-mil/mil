from __future__ import division

import numpy
import scipy
import scipy.signal
import math

from mil_passive_sonar.msg import Ping  # noqa


def resample(x, p, q):
    """Polyphase filter resample, based on MATLAB's resample.m"""
    bta = 5
    N = 10
    pqmax = max(p, q)
    fc = (1 / 2) / pqmax
    L = 2 * N * pqmax + 1
    h = p * scipy.signal.firwin(L - 1, 2 * fc, window=("kaiser", bta))
    Lhalf = (L - 1) / 2
    Lx = len(x)

    nz = int(math.floor(q - (Lhalf % q)))
    z = numpy.zeros(nz)  # noqa
    Lhalf += nz

    delay = math.floor(math.ceil(Lhalf) / q)
    nz1 = 0
    while math.ceil(((Lx - 1) * p + len(h) + nz1) / q) - delay < math.ceil(
        Lx * p / q
    ):  # noqa
        nz1 = nz1 + 1
    h = numpy.hstack([h, numpy.zeros(nz1)])
    y = upfirdn(x, h, p, q)
    Ly = math.ceil(Lx * p / q)
    y = y[int(delay) :]
    y = y[: int(Ly)]
    return y


def upfirdn(x, h, p, q):
    # Allocate an expanded array to hold x interspersed with p-1 zeros,
    # padded with enough zeros for the fir filter
    x_expanded = numpy.zeros((x.shape[0] - 1) * p + h.shape[0])

    # Insert x values every p elements
    x_expanded[: x.shape[0] * p : p] = x

    # Run filter
    x_filt = scipy.signal.lfilter(h, 1, x_expanded)
    return x_filt


def make_ping_channel(
    delay=0, freq=25e3, offset=0x7FFF, ampl=200, zeros=64, count=1024, sample_rate=300e3
):
    w = 2 * math.pi * freq / sample_rate
    sinwave = ampl * numpy.sin(w * (numpy.arange(count) - delay))

    delay_i = round(delay)
    window = numpy.zeros(count)
    window[zeros + delay_i :] = (
        numpy.minimum(numpy.exp(numpy.arange(count - zeros - delay_i) / 10), 2) - 1
    )

    noise = numpy.random.normal(0, 0.01, count)

    return offset + sinwave * window + noise


def make_ping(delays=[0, 0, 0, 0], args={}):
    return numpy.vstack(make_ping_channel(delay=delay, **args) for delay in delays)


def samples_to_list(samples):
    return samples.transpose().flatten().tolist()


def ping_to_samples(ping):
    return (
        numpy.array(ping.data, dtype=numpy.float64)
        .reshape((ping.samples, ping.channels))
        .transpose()
    )


def find_freq(data, rate):
    """Helper Function to find the freq of a signal
    *NOTE: Assumes the signal is strongest at the end and there is generally one dominant sin wave*

    data:
        numpy ndarray of shape (samples, channels)

    returns:
        averages the time between the last 5 sign changes on each channel to get an average freq of each channel
        averages the freq on each channel to return the final frequency

    """

    zero_crossings = numpy.zeros((5, data.shape[1]))
    for i in xrange(data.shape[1]):
        _zero_crossings = numpy.where(numpy.diff(numpy.sign(data[:, i]), axis=0))[0]
        if _zero_crossings.shape[0] < 5:
            raise Exception("not enough zero crossings to determine frequency")
        zero_crossings[:, i] = _zero_crossings[-5:]
        freq = float(rate) / (
            2 * numpy.mean(numpy.mean(numpy.diff(zero_crossings, axis=0), axis=0))
        )
    return freq


def find_freq_response(filt, rate, lower_freq, upper_freq, worN=2000):
    """Helper function to visualize the frequency response of a filter

    filt:
        filter

    rate:
        sample rate of the filter

    lower_freq:
        lowest frequency interested in (left bound of the crop)

    upper_freq:
        highest frequency interested in (right bound of the crop)

    worN:
        Optional: see `worN` at
            https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.freqz.html


    returns:
        x, y plot of frequency response of the filter fromllower_freq to upper_freq
    """
    w, h = scipy.signal.freqz(filt, [1], worN=worN)
    x = 0.5 * rate * w / numpy.pi
    start = numpy.min(numpy.where(x >= lower_freq)[0])
    end = numpy.min(numpy.where(x >= upper_freq)[0])
    x = x[start:end]
    y = 20 * numpy.log10(numpy.abs(h))
    y = y[start:end]
    return x, y


def calculate_dir_pinger(deltas, h_dist, v_sound):
    """
    deltas:
        numpy array of shape (4,) where datas[i] = time delay from h_0 to h_i

    h_dist:
        the unit distance from any hydrophone_i(i!=0) to hydrophone_0 in meters

    v_sound:
        speed of sound in water (meters / sec)


    returns:
        numpy array of unit vector towards the pinger (in hydrophone frame) *NOTE: the hydrophone
            array has symmetry along the z axis, so we always assume the pinger is below us*

    assumes hydrophone arrangment:
        h_0 = (0, 0, 0)
        h_1 = (1, 0, 0) * h_dist
        h_2 = (-1,0, 0) * h_dist
        h_3 = (0, 1, 0) * h_dist

    Design:

                 x_hat * delta_i * v_sound
                  |
                  \/
    (pressure |------>0  <-h_i
    wave)     |    __/
    x_hat---->| __/  <----h_dist
    (unit vec)|/
              0------->
             /     /\
        h_0 /      |
     (origin)    x_hat.h_i (dot product => scalar)


            x_hat.h_i = ||x_hat * delta_i * v_sound|| =>
            x_hat.h_i = delta_i * v_sound
    """
    if numpy.max(deltas) > h_dist / float(v_sound):
        raise Exception(
            "an impossible time delay was detected ( > %f)" % float(h_dist / v_sound)
        )
    x = numpy.zeros((3,))

    # average the values from h1 and h3 since on the same axis
    x[0] = -1 * (((deltas[1] * v_sound / h_dist) - (deltas[2] * v_sound / h_dist)) / 2)
    x[1] = deltas[3] * v_sound / h_dist

    x *= -1
    # since x is a unit vector and we know that the pinger is below us
    if x[0] ** 2 + x[1] ** 2 >= 1.0:
        x /= numpy.linalg.norm(x)
    else:
        x[2] = numpy.sqrt(1 - (x[0] ** 2 + x[1] ** 2))
    if True in numpy.isnan(x):
        raise Exception("nans: ", x)
    return -1 * x
