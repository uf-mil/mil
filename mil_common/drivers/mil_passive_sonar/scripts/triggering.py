#!/usr/bin/env python3
from typing import Union

import numpy as np
import rospy
from mil_passive_sonar import util
from mil_passive_sonar.msg import HydrophoneSamplesStamped, Ping, Triggered
from mil_passive_sonar.streamed_bandpass import StreamedBandpass
from mil_ros_tools import Plotter, interweave
from rospy.numpy_msg import numpy_msg
from scipy.ndimage.filters import maximum_filter1d
from std_msgs.msg import Header
from std_srvs.srv import (
    SetBool,
    SetBoolRequest,
    SetBoolResponse,
    Trigger,
    TriggerRequest,
    TriggerResponse,
)


class HydrophoneTrigger:
    """
    ROS Node meant to trigger only when a ping happens in our target frequency range.

    Subscribes to raw hydrophone samples on ``/samples``.

    Publishes samples (of gradient) from right around the triggering on ``/pings``
    when a ping is detected and found to be in the target frequency range.

    Optionally, you can publish a plot of the frequency response of the filter on `/filter_debug`
    by service calling `/filter_debug_trigger`.

    Attributes:
        general_lower (int): The lower bound of the frequency range. Set to the
            frequency of the lowest frequency pinger - 10 kHz.
        general_upper (int): The upper bound of the frequency range. Set to the
            frequency of the lowest frequency pinger - 10 kHz.
        time (float): The amount of time that has passed since recording started,
            according to the messages received.
        window_time (float): The maximum number of seconds to wait for another ping.
        pub (rospy.Publisher): A publisher for the ``/pings`` topic. Publishes
            :class:`Triggered` messages.
        sub (Optional[rospy.Subscriber]): A subscriber to the ``/samples`` topic. Receives
            either a :class:`~mil_passive_sonar.msg._Ping.Ping` or
            :class:`~mil_passive_sonar.msg._HydrophoneSamplesStamped.HydrophoneSamplesStamped` message,
            and sends the message to :meth:`.hydrophones_cb`.
        trigger_debug (Plotter): A plotter responsible for publishing debug data
            about the triggering behavior of the system.
    """

    def __init__(self):

        # Attributes about our general frequency range (all pinger live here)
        #  Frequency range garunteed to be relatively quiet except for the pingers (in Hz)
        self.general_lower = 15000  # lowest frequency pinger - 10 kHz
        self.general_upper = 50000  # highest frequency pinger + 10 kHz

        # Misc Attributes
        # time (from start of samples)
        self.time = 0.0
        #  max convolution window, the amount of time that we assume should be quiet before a ping in sec
        self.window_time = 0.1
        #  prev data buffer
        self.prev_data = None

        # Ros stuff
        rospy.init_node("hydrophone_trigger")
        # used for old bags
        msg_type = rospy.get_param("~sample_msg_type")
        if msg_type == "Ping":
            self.sub = rospy.Subscriber("samples", numpy_msg(Ping), self.hydrophones_cb)
        elif msg_type == "HydrophoneSamplesStamped":
            self.sub = rospy.Subscriber(
                "samples", numpy_msg(HydrophoneSamplesStamped), self.hydrophones_cb
            )
        else:
            ROS_FATAL("sample_msg_type not supported")
            return
        self.pub = rospy.Publisher("pings", numpy_msg(Triggered), queue_size=1)
        rospy.Service("~filter_debug_trigger", Trigger, self.filter_response)
        rospy.Service("~enable", SetBool, self.enable)
        rospy.Service("~reset", Trigger, self.reset)

        # Debug attributes
        #  Debug plot publishers
        self.trigger_debug = Plotter("~trigger_debug")
        self.sample_at_trigger_debug = Plotter("~sample_at_trigger_debug")
        self.filter_debug = Plotter("~filter_debug")

        self.get_params()

    def get_params(self):
        self.enabled = rospy.get_param("~enable_on_launch", True)

        # Attributes about our target frequency range
        #  target Frquency in Hz
        self.target = rospy.get_param("~target_frequency")
        #  tolerance around that frequerncy in Hz
        tolerance = rospy.get_param("~frequency_tolerance")
        # Filter attributes
        #  special filter property, do not change in Hz
        trans_width = 500
        #  Filter order for bandpass filter, (higher is better, but slower to compute)
        filt_order = 6000
        # reset bandpass filter
        self.bandpass_filter = StreamedBandpass(
            self.target - tolerance, self.target + tolerance, trans_width, filt_order
        )

        # Physical Properties
        # min time between pings (of any kind) in sec
        self.min_time_between_pings = rospy.get_param("~min_time_between_pings")
        self.prev_trigger_time = -1 * self.min_time_between_pings
        self.dist_h = rospy.get_param("dist_h")
        self.v_sound = rospy.get_param("v_sound")

        # Misc attributes
        #  minimum gradient of the max convolution wrt time to trigger a time of arivals calculation
        self.threshold = rospy.get_param("~threshold")
        self.trigger_offset = rospy.get_param("~trigger_offset")
        #  how far after the triggering time to make upper bound of samples at triggering in sec
        self.trigger_window_future = 1.0 * (self.dist_h / self.v_sound)
        #  how far before the triggering time to make lower bound of samples at triggering in sec
        self.trigger_window_past = 1.0 * (self.dist_h / self.v_sound)

    def reset(self, req: TriggerRequest) -> TriggerResponse:
        """
        Resets the system using the :meth`.get_params` method.

        Args:
            req (TriggerRequest): The request received.

        Returns:
            TriggerResponse: Whether the reset request was successful.
        """
        res = TriggerResponse()
        res.success = True
        self.get_params()
        return res

    def enable(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Enables the system; can be called through a :class:`SetBoolRequest` request.

        Args:
            req (SetBoolRequest): The service request.

        Returns:
            SetBoolResponse: The response to the service request.
        """
        self.enabled = req.data
        res = SetBoolResponse()
        res.success = True
        return res

    def filter_response(self, req: TriggerRequest) -> TriggerResponse:
        """
        Filters the debug service. Serves as a callback to the service.

        Args:
            req (TriggerRequest): The trigger request.

        Returns:
            TriggerResponse: The response to the request.
        """
        res = TriggerResponse()
        if self.bandpass_filter.h is None:
            res.message = "filter has not yet been created (created on the first callback after target being set)"
            res.success = False
            return res
        x, y = util.find_freq_response(
            self.bandpass_filter.h, self.rate, self.general_lower, self.general_upper
        )
        plots = np.vstack((x, y))
        titles = ["frequency (Hz)  vs Gain (dB)"]
        self.filter_debug.publish_plots(plots, titles=titles)
        res.success = True
        return res

    def hydrophones_cb(self, msg: Union[Ping, HydrophoneSamplesStamped]) -> None:
        """
        The callback to the hydrophone samples topic. Can receive either a :class:`~mil_passive_sonar.msg._Ping.Ping`
        or :class:`~mil_passive_sonar.msg._HydrophoneSamplesStamped.HydrophoneSamplesStamped` message.

        Args:
            msg (Union[Ping, HydrophoneSamplesStamped]): The message that can be
                received by the callback.
        """
        # Record start time of cb to make sure we are running in real time
        start_cb = rospy.get_rostime()
        if type(msg) == numpy_msg(Ping):
            msg_header = msg.header
            msg_channels = msg.channels
            msg_samples = msg.samples
            msg_sample_rate = msg.sample_rate
            msg_data = msg.data
        elif type(msg) == numpy_msg(HydrophoneSamplesStamped):
            msg_header = msg.header
            msg_channels = msg.hydrophone_samples.channels
            msg_samples = msg.hydrophone_samples.samples
            msg_sample_rate = msg.hydrophone_samples.sample_rate
            msg_data = msg.hydrophone_samples.data

        # if not enabled, go no further
        if not self.enabled:
            return

        if self.bandpass_filter.rate is None:
            self.rate = msg_sample_rate
            self.window_size = int(self.window_time * self.rate)
            self.bandpass_filter.rate = self.rate
            self.bandpass_filter.make_filter()

        # Resize the 1d array to be a 2d array of samples by channels
        msg_data.resize(msg_samples, msg_channels)

        # Apply a bandpass filter to the data for our general frequency range (on all channels)
        new_data = self.bandpass_filter.convolve(msg_data)
        # We always look at 2 messages of data at a time concatenated (0.1 sec each msg (for sylphase board))
        if self.prev_data is None:
            self.prev_data = new_data
            return
        else:
            data = np.concatenate((self.prev_data, new_data))
        # Time according to the passive sonar interface (assuming no missed messages)
        time = np.linspace(
            self.time, self.time + data.shape[0] / float(self.rate), data.shape[0]
        )

        # only use hydrophone 0 to trigger, more efficient
        # do a max convolution on the data
        max_convolves = np.apply_along_axis(
            lambda x: maximum_filter1d(x, self.window_size, axis=0), 0, data
        )
        max_convolve = max_convolves[:, 0]
        #  NOTE: No need to crop because zero padding is used and we are doing a max convolution

        # take the gradient of the max convolution (we are looking for steep increase = big gradient)
        gradients = np.gradient(max_convolves, axis=0)

        # print(np.max(gradients[:, 0]))
        # print(np.max(gradients[:, 0]), self.threshold)
        if np.max(gradients[:, 0]) >= self.threshold:

            triggered_at_idx = np.min(np.where(gradients[:, 0] >= self.threshold)[0])
            triggered_at_idx += int(self.trigger_offset * self.rate)
            trigger_time = time[triggered_at_idx]

            # if we have triggered very recently, do not trigger (echo protection)
            if trigger_time - self.prev_trigger_time > self.min_time_between_pings:

                self.prev_trigger_time = trigger_time

                start = triggered_at_idx - int(self.rate * self.trigger_window_past)
                end = triggered_at_idx + int(self.rate * self.trigger_window_future)

                ping_data = gradients[start:end]
                triggered_at_sample = triggered_at_idx + int((self.window_size - 1) / 2)
                start_sample = triggered_at_sample - int(
                    50 * self.rate * self.trigger_window_past
                )
                end_sample = triggered_at_sample + int(
                    50 * self.rate * self.trigger_window_future
                )
                ping_samples = data[start_sample:end_sample]

                try:
                    freq = util.find_freq(ping_samples, self.rate)
                except Exception as e:
                    rospy.logwarn("/hydrophone/triggering in find_freq %s" % e)
                    freq = self.target
                rospy.loginfo(f"triggered at {trigger_time:f} on {freq:f} Hz")

                ping = Triggered()
                ping.header = Header()
                ping.header.stamp = rospy.Time.now()
                ping.hydrophone_samples.channels = msg_channels
                ping.hydrophone_samples.samples = int(end - start)
                ping.hydrophone_samples.sample_rate = self.rate
                ping.hydrophone_samples.data = list(ping_data.flatten().astype(int))
                ping.trigger_time = -1 * (
                    trigger_time
                    - (
                        (
                            2 * trigger_time
                            - self.trigger_window_past
                            + self.trigger_window_future
                        )
                        / 2
                    )
                )
                self.pub.publish(ping)

                trigger_time_samples = (
                    triggered_at_sample / float(self.rate)
                ) + self.time

                if self.trigger_debug.is_go():
                    plot_data = data[:, 0]
                    titles = [
                        "time vs Gradient of Max convolve",
                        "time vs max_convolve (Window = %s sec)" % self.window_time,
                        "time vs hydrophone0 data",
                    ]
                    vlines = [trigger_time, trigger_time, trigger_time_samples]
                    plotable = np.vstack((gradients[:, 0], max_convolve, plot_data))
                    plots = interweave(time, plotable)
                    self.trigger_debug.publish_plots(plots, titles, vlines)

                if self.sample_at_trigger_debug.is_go():
                    plot_time = time[start_sample:end_sample]
                    plots = interweave(plot_time, ping_samples.transpose())
                    titles = ["h%i" % i for i in range(4)]
                    vline = [trigger_time_samples]
                    self.sample_at_trigger_debug.publish_plots(plots, titles, vline)

        self.time += msg_samples / float(self.rate)
        self.prev_data = new_data

        spare_time = (float(msg_samples) / self.rate) - (
            rospy.get_rostime() - start_cb
        ).to_sec()

        if spare_time < 0:
            rospy.logwarn(
                "Spare Time After Callback: %f, Running slower than real time"
                % spare_time
            )


if __name__ == "__main__":
    a = HydrophoneTrigger()
    rospy.spin()
