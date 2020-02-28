from __future__ import division

import itertools
import socket
import sys
import time

import numpy
import scipy.signal
from matplotlib import pyplot
from scipy.ndimage.filters import maximum_filter1d

fs = 1.2e6


def connect_to_samples(samples_per_callback):
    if len(sys.argv) == 3:
        host, port = sys.argv[1], int(sys.argv[2])
        s = socket.socket()
        s.connect((host, port))

        start_time = time.time()
        callbacks = 0

        buf = bytearray('\0'*(8*samples_per_callback))
        buf_pos = 0

        while True:
            count = s.recv_into(memoryview(buf)[buf_pos:])
            if not count:
                return

            buf_pos += count

            if buf_pos == len(buf):
                yield numpy.frombuffer(buf, numpy.dtype('h').newbyteorder('>')).reshape((samples_per_callback, 4)).T
                buf_pos = 0

                callbacks += 1
                dt = callbacks * samples_per_callback / fs
                expected_time = start_time + dt
                if time.time() - expected_time > 5 + .001 * dt:
                    print 'running slower than realtime!'
                if time.time() - expected_time < -(5 + .001 * dt):
                    print 'running faster than realtime??'
    elif len(sys.argv) == 2:
        filename = sys.argv[1]
        with open(filename, 'rb') as f:
            t0 = time.time() + 1
            for i in itertools.count():
                d = f.read(8*samples_per_callback)
                if len(d) != 8*samples_per_callback:
                    break
                dt = t0 + (i+1) * samples_per_callback / fs - time.time()
                if dt < 0:
                    print 'running slower than realtime!'
                else:
                    time.sleep(dt)
                yield numpy.frombuffer(d, numpy.dtype('h').newbyteorder('>')).reshape((samples_per_callback, 4)).T
    else:
        print 'usage: %s <host> <port> or %s <filename' % (
            sys.argv[0], sys.argv[0],)
        sys.exit(1)


def apply_along_first_axis(f, x):
    return numpy.array([f(x[i]) for i in xrange(x.shape[0])])


def filter(sample_generator, h):
    from_last = numpy.zeros((4, 0))
    for samples in sample_generator:
        assert samples.shape[1] > len(h)
        res = apply_along_first_axis(
            lambda x: scipy.signal.fftconvolve(x, h), samples)
        res[:, :from_last.shape[1]] += from_last
        yield res[:, :samples.shape[1]]
        from_last = res[:, samples.shape[1]:]


def _max_convolve_slow(x, n):
    assert n >= 1
    # equivalent, but less efficient
    return [max(x[i:i+n]) for i in xrange(0, len(x) - n + 1)]


def _max_convolve(x, n):
    return maximum_filter1d(x, n)[n-1 - (n-1)//2:len(x) - (n-1)//2]


for n in xrange(1, 100):
    x = numpy.random.randn(100)
    a = _max_convolve_slow(x, n)
    b = _max_convolve(x, n)
    if n == 1:
        assert numpy.all(a == x)
    assert numpy.all(b == a)


def triggering(sample_generator):
    # robosub pinger frequencies are in [25, 40] kHz
    f_low = 15e3  # should be at least 10 kHz below minimum pinger frequency
    f_high = 50e3  # should be at least 10 kHz above maximum pinger frequency
    transition_width = 10e3
    if 1:  # bandpass
        h = scipy.signal.remez(201, [0, f_low-transition_width, f_low,
                                     f_high, f_high+transition_width, 600e3], [0, 1, 0], Hz=fs)
    else:  # no bandpass
        h = numpy.array([1])
    if 1:  # notch out 40kHz
        # tweak filter length so that we have a bin on exactly the desired frequency with the desired width
        h = numpy.concatenate([h, numpy.zeros(21000-len(h))])
        H = numpy.fft.rfft(h)
        H_freq = numpy.fft.rfftfreq(len(h), 1/fs)
        k = min(xrange(len(H)), key=lambda i: abs(H_freq[i] - 40e3))
        assert H_freq[k] == 40e3
        #print 'width', H_freq[k] - H_freq[k-1]
        H[k] = 0
        h = numpy.fft.irfft(H)
    if 0:  # change to 1 to view filter response
        # zero pad to get more frequency resolution
        h2 = numpy.concatenate([h, numpy.zeros(len(h)*9)])
        freq, response = numpy.fft.rfftfreq(
            len(h2), 1/fs), numpy.abs(numpy.fft.rfft(h2))
        pyplot.semilogy(freq, response, 'b-')
        #freq, response = scipy.signal.freqz(h)
        #pyplot.semilogy(0.5*fs*freq/numpy.pi, numpy.abs(response), 'g-')
        pyplot.show()
    # comment me out to disable filter entirely
    sample_generator = filter(sample_generator, h)

    # thresholding looks at the signal like this:
    # ------------------------------------/\/\/\/\/\/\/\/\/\/
    #     <QUIET WINDOW> <IGNORED WINDOW> |
    #                                     trigger instant
    #                        |                         |
    #                         <----------> <---------->
    #                          before time  after time
    # and waits for a sample to trigger at that has magnitude THRESHOLD times the maximum magnitude in the quiet window

    QUIET_TIME = int(100e-3 * fs)
    IGNORE_TIME = int(1e-3 * fs)
    BEFORE = int(.35e-3 * fs)
    AFTER = int(.25e-3 * fs)
    DEAD_TIME = int(100e-3 * fs)
    THRESHOLD = 4

    history_needed = max(BEFORE, QUIET_TIME + IGNORE_TIME)

    sample_buf = numpy.zeros((4, 10))
    processed_end = history_needed
    offset = 0

    for samples in sample_generator:
        sample_buf = numpy.hstack([sample_buf, samples])
        del samples

        while True:
            processing_now = processed_end, sample_buf.shape[1] - AFTER
            #print 'now', processing_now
            if processing_now[1] - processing_now[0] <= 0:
                break

            samples_with_extra = sample_buf[:, processing_now[0] -
                                            (QUIET_TIME + IGNORE_TIME):processing_now[1]]
            samples_without_extra = sample_buf[:,
                                               processing_now[0]:processing_now[1]]
            assert numpy.all(
                samples_with_extra[:, QUIET_TIME + IGNORE_TIME:] == samples_without_extra)

            trigger_mag = THRESHOLD * \
                _max_convolve(
                    numpy.abs(samples_with_extra[0]), QUIET_TIME + 1)[:-IGNORE_TIME]
            assert trigger_mag.shape == (samples_without_extra.shape[1],)
            # pyplot.plot(trigger_mag)
            # pyplot.plot(samples_without_extra[0])
            # pyplot.show()
            triggered = numpy.nonzero(numpy.abs(samples_without_extra[0]) >= trigger_mag)[
                0] + processing_now[0]
            for y in triggered:
                assert processing_now[0] <= y < processing_now[1]
            if len(triggered):
                first_sample = triggered[0]
                assert first_sample + AFTER < sample_buf.shape[1]
                #print first_sample + AFTER, sample_buf.shape
                assert first_sample >= BEFORE
                assert sample_buf.shape[1] >= first_sample + AFTER
                this_ping = sample_buf[:, first_sample -
                                       BEFORE:first_sample+AFTER].copy()
                this_trigger_mag = sample_buf[:,
                                              first_sample-BEFORE:first_sample+AFTER].copy()
                assert this_ping.shape[1] == BEFORE + AFTER
                print 'triggered at', (offset+first_sample)/fs
                # pyplot.plot(this_ping.T)
                # pyplot.show()
                yield this_ping
                processed_end = first_sample+DEAD_TIME
            else:
                processed_end = processing_now[1]

        trim_start = min(sample_buf.shape[1], processed_end) - history_needed
        if trim_start > 0:
            assert sample_buf.shape[1] >= trim_start
            sample_buf = sample_buf[:, trim_start:]
            processed_end -= trim_start
            offset += trim_start
