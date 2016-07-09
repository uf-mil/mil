#!/usr/bin/python
from __future__ import division
import numpy as np
import numpy.linalg as la
from scipy.signal import resample, periodogram

from itertools import product

import rospy
import rosparam

from sub8_msgs.srv import Sonar, SonarResponse
from sub8_ros_tools import thread_lock, make_header
from sub8_alarm import AlarmBroadcaster
from multilateration import Multilaterator

import threading
import serial
import binascii
import struct
import time
import crc16
import sys

# temp
import matplotlib
matplotlib.use("WX")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
fig = plt.figure(0)
fig2 =plt.figure(1)
plt.ion()
plt.show()

lock = threading.Lock()


class Sub8Sonar():
    '''
    Smart sensor that provides high level ROS code with the location of pinger pulses detected with
    Jake Easterling's Hydrophone board.

    TODO: Add a function to try and reconnect to the serial port if we lose connection.
    TODO: Express pulse location in map frame
    '''
    def __init__(self, method, c, hydrophone_locations, port, baud=19200):
        rospy.init_node("sonar")

        alarm_broadcaster = AlarmBroadcaster()
        self.disconnection_alarm = alarm_broadcaster.add_alarm(
            name='sonar_disconnect',
            action_required=True,
            severity=0
        )
        self.packet_error_alarm = alarm_broadcaster.add_alarm(
            name='sonar_packet_error',
            action_required=False,
            severity=2
        )

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=None)
            self.ser.flushInput()        
        except Exception, e:
            print "\x1b[31mSonar serial  connection error:\n\t", e, "\x1b[0m"
            return None

        self.c = c
        self.hydrophone_locations = []
        for key in hydrophone_locations:
            sensor_location = np.array([hydrophone_locations[key]['x'], hydrophone_locations[key]['y'], hydrophone_locations[key]['z']])
            self.hydrophone_locations += [sensor_location]
        self.multilaterator = Multilaterator(hydrophone_locations, self.c, method) # speed of sound in m/s
        self.est_signal_freq_kHz = 0
        self._freq_sum = 0
        self._freq_samples = 0

        rospy.Service('~/sonar/get_pinger_pulse', Sonar, self.get_pulse_from_raw_data)
        print "\x1b[32mSub8 sonar driver initialized\x1b[0m"
        rospy.spin()

    @thread_lock(lock)
    def get_pulse_from_timestamps(self, srv):
        self.ser.flushInput()

        try:
            self.ser.write('A')
            print "Sent timestamp request..."
        except:  # Except only serial errors in the future.
            self.disconnection_alarm.raise_alarm(
                problem_description="Sonar board serial connection has been terminated."
            )
            return False
        return self.multilaterator.getPulseLocation(self.timestamp_listener())

    def timestamp_listener(self):
        '''
        Parse the response of the board.
        '''
        print "Listening..."

        # We've found a packet now disect it.
        response = self.ser.read(19)
        # rospy.loginfo("Received: %s" % response) #uncomment for debugging
        if self.check_CRC(response):
            delete_last_lines(0) # Heard response!
            # The checksum matches the data so split the response into each piece of data.
            # For info on what these letters mean: https://docs.python.org/2/library/struct.html#format-characters
            data = struct.unpack('>BffffH', response)
            timestamps = [data[4], data[1], data[2], data[3] ]
            print "timestamps:", timestamps
            return timestamps
        else:
            self.packet_error_alarm.raise_alarm(
                problem_description="Sonar board checksum error.",
                parameters={
                    'fault_info': {'data': response}
                }
            )
            return None

    @thread_lock(lock)
    def get_pulse_from_raw_data(self, srv):
        # request signals from hydrophone board
        self.ser.flushInput()
        try:
            self.ser.write('B')
            print "Sent raw data request..."
        except:  # Except only serial errors in the future.
            self.disconnection_alarm.raise_alarm(
                problem_description="Sonar board serial connection has been terminated."
            )
            return False
        return self.multilaterator.getPulseLocation(self.raw_data_listener())

    def raw_data_listener(self):
        '''
        Parse the response of the board.
        '''
        print "Listening..."

        # prepare arrays for storing signals
        signal_bias = 32767
        waves_recorded = 3
        samples_per_wave = 17.24
        upsample_scale = 3
        exp_recording_size = np.floor(samples_per_wave) * waves_recorded * upsample_scale - 1
        raw_signals = np.zeros([4, exp_recording_size], dtype=float)

        try:
            # read in start bit of packet
            timeout = 0
            start_bit = self.ser.read(1)
            # If start bit not read, keep trying
            while start_bit != '\xBB':
                start_bit = self.ser.read(1)
                timeout += 1
                if timeout > 600:
                    raise BufferError("Timeout: failed to read a start bit from sonar board")
            for elem in np.nditer(raw_signals, op_flags=['readwrite']):
                elem[...] = float(self.ser.read(5)) - signal_bias  # ... is idx to current elem
        except BufferError as e:
            print e

        non_zero_end_idx = 57
        up_factor = 8
        sampling_freq = 430E3 * 0.8 # Hz
        samp_period = 1E6 / sampling_freq  # microseconds
        upsamp_step = samp_period / up_factor

        # Set how much of each signal to actually use for processing
        raw_signals = raw_signals[:, 0:non_zero_end_idx]

        # upsample the signals for successful cross-correlation
        upsampled_signals = [resample(x, up_factor*len(x)) for x in raw_signals]

        # scale waves so they all have the same variance
        equalized_signals = [x / np.std(x) for x in upsampled_signals]
        raw_signals = [x / np.std(x) for x in raw_signals]
        w0_upsamp, w1_upsamp, w2_upsamp, w3_upsamp = equalized_signals
        t_up = np.arange(0, non_zero_end_idx*samp_period, upsamp_step)

        zero_crossings = np.where(np.diff(np.sign(w0_upsamp)))[0]
        num_crossings = len(zero_crossings)
        if num_crossings % 2 == 0:  # we want an odd number of zero crossings
            zero_crossings = zero_crossings[1:]
            num_crossings -= 1
        waves_between_first_and_last_crossing = (num_crossings - 1) / 2
        time_between_first_and_last_crossing = t_up[zero_crossings[-1]] - t_up[zero_crossings[0]]
        curr_signal_freq_kHz = 1E3 * waves_between_first_and_last_crossing / time_between_first_and_last_crossing # kHz
        self._freq_sum += curr_signal_freq_kHz
        self._freq_samples += 1
        self.est_signal_freq_kHz = self._freq_sum / self._freq_samples


        # frequencies, spectrum = periodogram(w0_upsamp, sampling_freq * up_factor)
        # signal_freq = frequencies[np.argmax(spectrum)]  # Hz
        # print "fft source frequency:", signal_freq, "Hz"
        print "current source frequency:", curr_signal_freq_kHz, "kHz"
        print "est source frequency:", self.est_signal_freq_kHz, "kHz"
        # ax = fig2.add_subplot(111)
        # ax.semilogy(frequencies,spectrum)
        # ax.set_ylim([1e-7, 1e2])
        # ax.set_xlim([0, 6e4])
        # ax.set_xlabel('frequency [Hz]')
        signal_period = 1E3 / self.est_signal_freq_kHz  # microseconds
        upsamples_recorded = len(w0_upsamp)

        waves_ref = 3.5
        waves_non_ref = 2.5
        samples_per_wave = signal_period / samp_period
        ref_upsamples = int(round(waves_ref * samples_per_wave * up_factor))
        nonref_upsamples = int(np.ceil(waves_non_ref * samples_per_wave * up_factor))

        ref_start_idx = None
        if (len(w0_upsamp) % 2 == ref_upsamples % 2):
            ref_start_idx = int(round((upsamples_recorded / 2) - (ref_upsamples / 2)))
        else:
            ref_start_idx = int(np.ceil((upsamples_recorded / 2) - (ref_upsamples / 2)))
        ref_end_idx = ref_start_idx + ref_upsamples - 1
        nonref_end_idx = ref_start_idx + nonref_upsamples - 1

        w0_select = w0_upsamp[ref_start_idx : ref_end_idx + 1]
        w1_select = w1_upsamp[ref_start_idx : nonref_end_idx + 1]
        w2_select = w2_upsamp[ref_start_idx : nonref_end_idx + 1]
        w3_select = w3_upsamp[ref_start_idx : nonref_end_idx + 1]
        t_ref_select = t_up[ref_start_idx : ref_end_idx + 1]
        t_nonref_select = t_up[ref_start_idx : nonref_end_idx + 1]

        cc1 = np.correlate(w0_select, w1_select, mode='full')
        cc2 = np.correlate(w0_select, w2_select, mode='full')
        cc3 = np.correlate(w0_select, w3_select, mode='full')
        corr_start = t_ref_select[0] - t_nonref_select[-1]
        corr_end = t_ref_select[-1] - t_ref_select[0] + upsamp_step
        t_corr = np.arange(corr_start, corr_end, upsamp_step)
        print len(cc1), len(t_corr)

        fig.clf()
        ax1 = fig.add_subplot(511)
        ax2 = fig.add_subplot(512)
        ax3 = fig.add_subplot(513)
        ax4 = fig.add_subplot(514)
        ax5 = fig.add_subplot(515)
        axarr = [ax1, ax2, ax3, ax4, ax5]

        axarr[0].plot(t_up,w1_upsamp,'r',t_up,w2_upsamp,'g',t_up,w3_upsamp,'b',t_up,w0_upsamp,'k')
        axarr[0].axis([0,60*samp_period,-3,3])
        axarr[0].set_title('Signals')

        axarr[1].plot(t_nonref_select, w1_select, 'r',
                      t_nonref_select, w2_select, 'g',
                      t_nonref_select, w3_select, 'b',
                      t_ref_select,    w0_select, 'k')
        axarr[1].set_title('Selected portions')

        axarr[2].plot(t_corr, cc1)
        axarr[2].set_title('Hydrophone 1 cross-correlation')
        axarr[3].plot(t_corr, cc2)
        axarr[3].set_title('Hydrophone 2 cross-correlation')
        axarr[4].plot(t_corr, cc3)
        axarr[4].set_title('Hydrophone 3 cross-correlation')

        plt.draw()
        plt.pause(0.1)

        return [0,0,0,0] #DBG

    def max_delta_t(hydrophone_idx_a, hydrophone_idx_b):
        a = self.hydrophone_locations[hydrophone_idx_a]
        b = self.hydrophone_locations[hydrophone_idx_b]
        dist = la.norm(a - b)
        return dist / self.c

    def CRC(self, message):
        # You may have to change the checksum type.
        # Check the crc16 module online to see how to do that.
        crc = crc16.crc16xmodem(message, 0xFFFF)
        return struct.pack('>H', crc)

    def check_CRC(self, message):
        '''
        Given a message with a checksum as the last two bytes, this will return True or False
        if the checksum matches the given message.
        '''
        msg_checksum = message[-2:]
        raw_message = message[:-2]
        crc = crc16.crc16xmodem(raw_message, 0xFFFF)

        # If the two match the message was correct
        if crc == struct.unpack('>H', msg_checksum)[0]:
            return True
        else:
            return False

def delete_last_lines(n=1):
    CURSOR_UP_ONE = '\x1b[1A'
    ERASE_LINE = '\x1b[2K'
    for _ in range(n):
        sys.stdout.write(CURSOR_UP_ONE)
        sys.stdout.write(ERASE_LINE)

if __name__ == "__main__":
    d = Sub8Sonar('LS', 1.484, rospy.get_param('~/sonar_driver/hydrophones'),
                  "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH02X4IE-if00-port0",
                  19200)