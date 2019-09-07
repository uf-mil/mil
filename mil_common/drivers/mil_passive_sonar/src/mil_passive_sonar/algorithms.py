from __future__ import division
from mil_passive_sonar import util

import numpy
import scipy
import math
import sys
import matplotlib.pyplot as plt

DEBUG = 0

def run(samples, sample_rate, v_sound, dist_h, dist_h4):
    # Perform algorithm
    if DEBUG: plt.cla()
    if DEBUG: plt.subplot(2, 2, 1)
    if DEBUG: map(plt.plot, samples)
    samples = zero_mean(samples)
    freq, amplitude, samples_fft = compute_freq(
        samples, sample_rate, numpy.array([5e3, 40e3]), plot=True)
    fft_sharpness = amplitude**2 / numpy.sum(samples_fft)
    if DEBUG: plt.subplot(2, 2, 2)
    if DEBUG: map(plt.plot, samples_fft)
    upsamples, upsample_rate = preprocess(samples, sample_rate, 3e6)
    deltas, delta_errors, template_pos, template_width = \
        compute_deltas(upsamples, upsample_rate, max(dist_h, dist_h4) / v_sound, 20e-2 / v_sound)
    delta_errors = delta_errors / amplitude
    if len(deltas) == 3:
        pos = compute_pos_4hyd(deltas, upsample_rate, v_sound, dist_h, dist_h4)
    else:
        pos = numpy.empty(0)

    # Check for errors
    errors = []
    if amplitude < 80:
        errors.append('Low amplitude at maximum frequency')
    if template_pos is None:
        errors.append('Failed to find template')
    elif numpy.max(delta_errors) > 1e-3:
        errors.append('High template match error (%s)' % str(delta_errors))

    return dict(
        errors=errors,
        freq=freq,
        amplitude=amplitude,
        fft_sharpness=fft_sharpness,
        samples_fft=samples_fft,
        pos=pos,
        deltas=deltas,
        delta_errors=delta_errors,
        upsamples=upsamples,
        upsample_rate=upsample_rate,
        template_pos=template_pos,
        template_width=template_width)


def zero_mean(samples):
    """Zero means data by assuming that the first 32 samples are zeros"""
    return samples - numpy.mean(samples[:, 0:32], 1)[:, numpy.newaxis]


def normalize(samples):
    """Rescapes samples so each individual channel lies between -1 and 1"""
    return samples / numpy.amax(numpy.abs(samples), 1)[:, numpy.newaxis]


def compute_freq(samples, sample_rate, freq_range, plot=False):
    """Checks whether samples are likely a solid ping and returns the frequency."""
    samples_window = samples * numpy.hamming(samples.shape[1])

    # Compute fft, find peaks in desired range
    fft_length = samples.shape[1]
    samples_fft = numpy.absolute(
        numpy.fft.fft(samples_window, fft_length,
                      axis=1))[:, :int(fft_length / 2)]
    bin_range = freq_to_bin(freq_range, sample_rate, fft_length).astype(int)
    peaks = bin_range[0] + numpy.argmax(
        samples_fft[:, bin_range[0]:bin_range[1]], axis=1)

    # Sort peaks, take mean of the middle
    middle_peaks = numpy.sort(peaks)[len(peaks) // 4:
                                     len(peaks) - len(peaks) // 4]
    peak = numpy.mean(middle_peaks)

    freq = bin_to_freq(peak, sample_rate, fft_length)

    amplitude = math.sqrt(numpy.mean(samples_fft[:, int(round(peak))]))
    return freq, amplitude, samples_fft


def bin_to_freq(bin, sample_rate, fft_length):
    return (sample_rate / 2) / (fft_length / 2) * bin


def freq_to_bin(freq, sample_rate, fft_length):
    return freq * ((fft_length / 2) / (sample_rate / 2))


def preprocess(samples, sample_rate, desired_sample_rate):
    """Upsamples data to have approx. desired_sample_rate."""
    samples = bandpass(samples, sample_rate)
    samples = normalize(samples)

    # Upsample each channel
    upfact = int(round(desired_sample_rate / sample_rate))
    upsamples = numpy.empty((samples.shape[0], samples.shape[1] * upfact))
    for i in xrange(samples.shape[0]):
        upsamples[i, :] = util.resample(samples[i, :], upfact, 1)
    return upsamples, sample_rate * upfact


def bandpass(samples, sample_rate):
    """Applies a 20-30khz bandpass FIR filter"""
    # 25-40KHz is the range of the pinger for the roboboat competition
    fir = scipy.signal.firwin(
        128, [19e3 / (sample_rate / 2), 41e3 / (sample_rate / 2)],
        window='hann',
        pass_zero=False)
    return scipy.signal.lfilter(fir, 1, samples)


def compute_deltas(samples,
                   sample_rate,
                   max_delay,
                   template_duration,
                   plot=False):
    """
    Computes N-1 position deltas for N channels, by making a template
    for the first channel and matching to all subsequent channels.
    """
    template_width = int(round(template_duration * sample_rate))
    template, template_pos = make_template(samples[0, :], template_width)
    if DEBUG: plt.subplot(2, 2, 3)
    if DEBUG: plt.plot(template)
    if template_pos is None:
        return numpy.empty(0), numpy.empty(0), None, template_width
    start = template_pos - int(round(max_delay * sample_rate * 1.25))
    stop = template_pos + int(round(max_delay * sample_rate * 1.25))

    deltas = numpy.empty(samples.shape[0] - 1)
    errors = numpy.empty(samples.shape[0] - 1)
    if DEBUG: plt.subplot(2, 2, 4)
    for i in xrange(1 + deltas.shape[0]):
        res = match_template(samples[i, :], start, stop, template)
        if res is None:
            return numpy.empty(0), numpy.empty(0), None, template_width
        pos, error = res
        if i >= 1:
            i -= 1
            deltas[i] = pos - template_pos
            errors[i] = error
    if DEBUG: plt.show()

    return deltas, errors, template_pos, template_width


def make_template(channel, width):
    """
    Returns a template of the specified width, with its 25% position being at
    where the lower-level driver triggered.
    """
    pos = int(round(channel.shape[0] * .35/(.35+.25) - width*.25))
    return channel[pos:pos + width], pos


def match_template(channel, start, stop, template):
    """
    Matches template to channel, returning the point where the start
    of the template should be placed.
    """
    assert start >= 0
    start = max(start, 0)
    assert stop <= channel.shape[0] - template.shape[0]
    stop = min(stop, channel.shape[0] - template.shape[0])
    err = calculate_error(channel, start, stop, template)
    if DEBUG: plt.plot(err)
    min_pt = find_minimum(err)
    if min_pt is None: return None

    return start + min_pt, err[int(round(min_pt))]


def calculate_error(channel, start, stop, template):
    """
    Slides the template along channel from start to stop, giving the
    error of the template and channel at each
    location.
    """
    width = template.shape[0]
    res = numpy.zeros(stop - start)
    for i in xrange(start, stop):
        # used to use mean absolute difference; now using (negative) pearson
        # correlation coefficient to be invariant to scale/shift
        #res[i - start] = numpy.mean(numpy.abs(channel[i:i + width] - template))
        res[i - start] = -numpy.corrcoef(channel[i:i + width], template)[0, 1]
    return res


def find_minimum(data):
    """
    Finds the sub-sample position of the minimum in a continuous signal,
    by first finding the lowest absolute value, then doing quadratic interpolation.
    """
    pos = numpy.argmin(data)
    if pos == 0 or pos == len(data) - 1:
        print 'warning: pos on border; search region not large enough?'
        return None
    yl, yc, yr = data[pos-1], data[pos], data[pos+1]
    pos += (yr-yl)/(4*yc-2*yl-2*yr)
    return pos


def compute_pos_4hyd(deltas, sample_rate, v_sound, dist_h, dist_h4):
    """
    Solves the 4 hydrophone case (3 deltas) for heading, declination,
    and sph_dist using a bunch of trig. Future work would be to phrase
    this as a NLLSQ problem or something, and adapt it to more
    hydrophones.
    """
    assert (len(deltas) == 3)

    y1 = deltas[0] / sample_rate * v_sound
    y2 = deltas[1] / sample_rate * v_sound
    y3 = deltas[2] / sample_rate * v_sound

    dist = abs((y1**2 + y2**2 - 2 * dist_h**2) / (2 * y1 + 2 * y2))
    cos_alpha1 = (2 * dist * y1 + y1**2 - dist_h**2) / (-2 * dist * dist_h)
    cos_alpha2 = -(2 * dist * y2 + y2**2 - dist_h**2) / (-2 * dist * dist_h)
    cos_alpha = (cos_alpha1 + cos_alpha2) / 2

    cos_beta = (2 * dist * y3 + y3**2 - dist_h4**2) / (-2 * dist * dist_h4)

    dist_x = cos_alpha * dist
    dist_y = cos_beta * dist
    if dist**2 - (dist_x**2 + dist_y**2) < 0:
        dist_z = 0
    else:
        dist_z = -math.sqrt(dist**2 - (dist_x**2 + dist_y**2))
    return numpy.array([dist_x, dist_y, dist_z])


if __name__ == '__main__':
    sample_rate = 300e3
    if len(sys.argv) > 1:
        samples = numpy.loadtxt(sys.argv[1], delimiter=',').transpose()
    else:
        samples = util.make_ping([0, .25, 1.234, -5], {
            'freq': 23e3,
            'sample_rate': sample_rate
        })

    r = run(samples, sample_rate, 1497, 2.286000e-02, 2.286000e-02)

    if len(r['errors']) > 0:
        print 'ERRORS', r['errors']
    print 'freq', r['freq'], 'amplitude', r['amplitude']
    print 'sharpness', r['fft_sharpness']
    print 'deltas', r['deltas']
    print 'delta errors', r['delta_errors'] / r['amplitude']
    print 'pos (hyd coordinates)', r['pos']

    plt.figure()
    plt.plot(samples.transpose())
    plt.title('Raw ping')

    plt.figure()
    fft_length = r['samples_fft'].shape[1] * 2
    plt.plot(
        bin_to_freq(numpy.arange(0, fft_length // 2), sample_rate, fft_length),
        r['samples_fft'].transpose())
    plt.title('FFT')

    plt.figure()
    plt.plot(r['upsamples'].transpose())
    plt.title('Upsampled ping')

    if r['template_pos'] is not None:
        period = int(round(r['upsample_rate'] / r['freq']))
        template = r['upsamples'][0, r['template_pos']:
                                  r['template_pos'] + r['template_width']]
        plot_start = r['template_pos'] - 2 * period
        plot_stop = r['template_pos'] + r['template_width'] + 2 * period
        plt.ioff()
        plt.figure()
        plt.plot(template)
        plt.title('Template')

        for i in xrange(r['deltas'].shape[0]):
            plt.figure()
            plt.plot(
                numpy.arange(plot_start, plot_stop),
                r['upsamples'][i + 1, plot_start:plot_stop])
            pos = r['template_pos'] + int(round(r['deltas'][i]))
            plt.plot(numpy.arange(pos, pos + r['template_width']), template)
            plt.title('Channel %d' % (i + 1))

    plt.show()
