import numpy as np
import scipy
from scipy.signal import fftconvolve


class StreamedBandpass(object):
    '''Class for bandpass filtering sequential batches of data.

    Feature: takes care of the overlap-adding that needs to be done 
               when filtering sequential batches of data indivudually

    '''
    def __init__(self, lower, upper, trans_width, order, rate=None):
        self.from_prev = None
        self.size = order
        self.h = None
        self.lower = lower
        self.upper = upper
        self.trans_width = trans_width
        self.rate = rate

        if self.is_ready_to_make_filter():
            self.make_filter()


    def is_ready_to_make_filter(self):
        return self.lower is not None and\
               self.upper is not None and\
               self.trans_width is not None and\
               self.rate is not None and\
               self.size is not None


    def make_filter(self):
        if not self.is_ready_to_make_filter():
            raise Exception('Some parameters have not been filled in yet: ', self.__dict__)
        self.h = scipy.signal.remez(self.size,
                [0, self.lower - self.trans_width,
                self.lower, self.upper,
                self.upper + self.trans_width, self.rate/2], [0,1,0], Hz = self.rate)


    def convolve(self, data):
        '''Does the filtering

            data: nd array of shape (samples, channels)

        '''
        filtered_data = np.apply_along_axis(lambda x: fftconvolve(x, self.h, 'full'), 0, data)
        if self.from_prev is not None:
            filtered_data[:self.size - 1] += self.from_prev
        self.from_prev = filtered_data[-(self.size-1):]

        return filtered_data[:-(self.size-1)]
