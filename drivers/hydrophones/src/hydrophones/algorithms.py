from __future__ import division
from hydrophones import util

import numpy
import scipy
import math
import sys
import matplotlib.pyplot as plt

def run(samples, sample_rate, v_sound, dist_h, dist_h4):
    # Perform algorithm
    samples = zero_mean(samples)
    freq, amplitude, samples_fft = compute_freq(samples, sample_rate, numpy.array([5e3, 40e3]), plot=True)
    fft_sharpness = amplitude**2/numpy.sum(samples_fft)
    upsamples, upsample_rate = preprocess(samples, sample_rate, 3e6)
    deltas, delta_errors, template_pos, template_width = \
        compute_deltas(upsamples, upsample_rate, freq, 8)
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
    samples_fft = numpy.absolute(numpy.fft.fft(samples_window, fft_length, axis=1))[:, :fft_length/2]
    bin_range = freq_to_bin(freq_range, sample_rate, fft_length)
    peaks = bin_range[0] + numpy.argmax(samples_fft[:, bin_range[0]:bin_range[1]], axis=1)
    
    # Sort peaks, take mean of the middle
    middle_peaks = numpy.sort(peaks)[len(peaks)//4:len(peaks) - len(peaks)//4]
    peak = numpy.mean(middle_peaks)

    freq = bin_to_freq(peak, sample_rate, fft_length)

    amplitude = math.sqrt(numpy.mean(samples_fft[:, round(peak)]))
    return freq, amplitude, samples_fft

def bin_to_freq(bin, sample_rate, fft_length):
    return (sample_rate/2) / (fft_length/2) * bin

def freq_to_bin(freq, sample_rate, fft_length):
    return freq * ((fft_length/2) / (sample_rate/2))

def preprocess(samples, sample_rate, desired_sample_rate):
    """Upsamples data to have approx. desired_sample_rate."""
    # Trim to first ms of data and bandpass
    samples = bandpass(samples[:, :.001*sample_rate], sample_rate)
    samples = normalize(samples)

    # Upsample each channel
    upfact = round(desired_sample_rate/sample_rate)
    upsamples = numpy.empty((samples.shape[0], samples.shape[1]*upfact))
    for i in xrange(samples.shape[0]):
        upsamples[i, :] = util.resample(samples[i, :], upfact, 1)
    return upsamples, sample_rate*upfact

def bandpass(samples, sample_rate):
    """Applies a 20-30khz bandpass FIR filter"""
    # 25-40KHz is the range of the pinger for the roboboat competition
    fir = scipy.signal.firwin(128,
                              [19e3/(sample_rate/2), 41e3/(sample_rate/2)],
                              window='hann',
                              pass_zero=False)
    return scipy.signal.lfilter(fir, 1, samples)

def compute_deltas(samples, sample_rate, ping_freq, template_periods, plot=False):
    """
    Computes N-1 position deltas for N channels, by making a template
    for the first channel and matching to all subsequent channels.
    """
    period = int(round(sample_rate / ping_freq))
    template_width = period*template_periods+1
    template, template_pos = make_template(samples[0, :],
                                           .2,
                                           template_width)
    if template_pos is None:
        return numpy.empty(0), numpy.empty(0), None, template_width
    start = template_pos - period//2
    stop = template_pos + period//2

    deltas = numpy.empty(samples.shape[0]-1)
    errors = numpy.empty(samples.shape[0]-1)
    for i in xrange(deltas.shape[0]):
        pos, error = match_template(samples[i+1, :], start, stop, template)
        deltas[i] = pos - template_pos
        errors[i] = error

    return deltas, errors, template_pos, template_width

def make_template(channel, thresh, width):
    """
    Returns a template of the specified width, centered at the first
    point of the channel to exceed thresh.
    """
    for pos in xrange(0, channel.shape[0]-width):
        if abs(channel[pos+width//4]) >= thresh:
            return channel[pos:pos+width], pos
    return None, None

def match_template(channel, start, stop, template):
    """
    Matches template to channel, returning the point where the start
    of the template should be placed.
    """
    start = max(start, 0)
    stop = min(stop, channel.shape[0] - template.shape[0])
    mad = mean_absolute_difference(channel, start, stop, template)
    min_pt = find_zero(mad)

    return start + min_pt, mad[round(min_pt)]

def mean_absolute_difference(channel, start, stop, template):
    """
    Slides the template along channel from start to stop, giving the
    mean absolute difference of the template and channel at each
    location.
    """
    width = template.shape[0]
    mad = numpy.zeros(stop-start)
    for i in xrange(start, stop):
        mad[i-start] = numpy.mean(numpy.abs(channel[i:i+width] - template))
    return mad

def find_zero(data):
    """
    Finds the sub-sample position of the first zero in a continuous signal,
    by first finding the lowest absolute value, then taking the gradient
    and performing a linear interpolation near the lowest absolute value.
    """
    approx = numpy.argmin(numpy.abs(data))
    d_data = numpy.gradient(data)

    for pos in xrange(max(approx-3,0), min(approx+3, d_data.shape[0]-1)):
        if numpy.sign(d_data[pos]) != numpy.sign(d_data[pos+1]):
            y2 = d_data[pos+1]
            y1 = d_data[pos]
            x2 = pos+1
            x1 = pos
            return -(x2-x1)/(y2-y1)*y1 + x1
    return approx

def compute_pos_4hyd(deltas, sample_rate, v_sound, dist_h, dist_h4):
    """
    Solves the 4 hydrophone case (3 deltas) for heading, declination,
    and sph_dist using a bunch of trig. Future work would be to phrase
    this as a NLLSQ problem or something, and adapt it to more
    hydrophones.
    """
    assert(len(deltas) == 3)

    y1 = deltas[0]/sample_rate*v_sound
    y2 = deltas[1]/sample_rate*v_sound
    y3 = deltas[2]/sample_rate*v_sound

    dist = abs((y1**2 + y2**2 - 2*dist_h**2)/(2*y1 + 2*y2))
    cos_alpha1 = (2*dist*y1 + y1**2 - dist_h**2)/(-2*dist*dist_h);
    cos_alpha2 = -(2*dist*y2 + y2**2 - dist_h**2)/(-2*dist*dist_h);
    cos_alpha = (cos_alpha1 + cos_alpha2)/2;

    cos_beta = (2*dist*y3 + y3**2 - dist_h4**2)/(-2*dist*dist_h4);

    dist_x = cos_alpha*dist
    dist_y = cos_beta*dist
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
        samples = util.make_ping([0, .25, 1.234, -5], {'freq': 23e3, 'sample_rate': sample_rate})

    r = run(samples, sample_rate, 1497, 2.286000e-02, 2.286000e-02)

    if len(r['errors']) > 0:
        print 'ERRORS', r['errors']
    print 'freq', r['freq'], 'amplitude', r['amplitude']
    print 'sharpness', r['fft_sharpness']
    print 'deltas', r['deltas']
    print 'delta errors', r['delta_errors']/r['amplitude']
    print 'pos (hyd coordinates)', r['pos']

    plt.figure()
    plt.plot(samples.transpose())
    plt.title('Raw ping')

    plt.figure()
    fft_length = r['samples_fft'].shape[1]*2
    plt.plot(bin_to_freq(numpy.arange(0, fft_length//2), sample_rate, fft_length), 
             r['samples_fft'].transpose())
    plt.title('FFT')
    
    plt.figure()
    plt.plot(r['upsamples'].transpose())
    plt.title('Upsampled ping')

    if r['template_pos'] is not None:
        period = int(round(r['upsample_rate'] / r['freq']))
        template = r['upsamples'][0,r['template_pos']:r['template_pos']+r['template_width']]
        plot_start = r['template_pos'] - 2*period        
        plot_stop = r['template_pos'] + r['template_width'] + 2*period
        plt.ioff()
        plt.figure()
        plt.plot(template)
        plt.title('Template')

        for i in xrange(r['deltas'].shape[0]):
            plt.figure()
            plt.plot(numpy.arange(plot_start, plot_stop), r['upsamples'][i+1, plot_start:plot_stop])
            pos = r['template_pos'] + int(round(r['deltas'][i]))
            plt.plot(numpy.arange(pos, pos+r['template_width']), template)
            plt.title('Channel %d' % (i+1))

    plt.show()


