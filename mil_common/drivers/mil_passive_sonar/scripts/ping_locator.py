#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped
from mil_passive_sonar import util
from mil_passive_sonar.msg import Triggered
from mil_ros_tools import Plotter, interweave
from mil_tools import numpy_to_vector3
from rospy.numpy_msg import numpy_msg
from scipy.signal import correlate
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


class PingLocator:
    """
    Subscribers to ``/pings``, and publishes vector to the pinger on ``/direction``.
    Creates a node named ``ping_locator``.
    """

    def __init__(self):
        rospy.init_node("ping_locator")
        self.sub = rospy.Subscriber("pings", numpy_msg(Triggered), self.ping_cb)
        self.pub = rospy.Publisher("direction", Vector3Stamped, queue_size=10)

        rospy.Service("~enable", SetBool, self.enable)

        self.enabled = True

        self.debug = Plotter("~cross_correlation_debug")
        self.debug_samples = Plotter("~samples_debug")

        self.dist_h = rospy.get_param("dist_h")
        self.v_sound = rospy.get_param("v_sound")

    def enable(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Sets the locator to enabled. Doesn't immediately do anything.

        Args:
            req (SetBoolRequest): The service request.

        Returns:
            SetBoolResponse: The service response.
        """
        self.enabled = req.data
        res = SetBoolResponse()
        res.success = True
        return res

    def ping_cb(self, msg) -> None:
        """
        Callback called with each new ping. With each call, creates a
        :class:`~geometry_msgs.msg._Vector3Stamped.Vector3Stamped` message containing
        the direction of the pinger.

        Args:
            msg (Triggered): The received message, with numpy formatting.
        """
        data = msg.hydrophone_samples.data
        data.resize((msg.hydrophone_samples.samples, msg.hydrophone_samples.channels))
        rate = msg.hydrophone_samples.sample_rate

        cross_corr = np.array(
            [
                correlate(data[:, 0], data[:, i], mode="same")
                for i in range(0, data.shape[1])
            ],
        ).transpose()

        total_time = float(cross_corr.shape[0]) / rate
        time = np.linspace(total_time / -2, total_time / 2, cross_corr.shape[0])

        deltas = np.zeros((4,))

        maxes = np.array(
            [np.argmax(cross_corr[:, i]) for i in range(cross_corr.shape[1])],
        )

        if (maxes == cross_corr.shape[0] - 1).any() or (maxes == 0).any():
            rospy.logerr(
                "/hydrophones/ping_locator lack of features on one of the channels",
            )
            return

        deltas = np.array(time[maxes])
        deltas -= deltas[0]

        plots = interweave(time, cross_corr.transpose())
        titles = ["h0 cross h%d" % i for i in range(data.shape[0])]
        self.debug.publish_plots(plots, titles, time[maxes])

        plots = interweave(time, data.transpose())
        titles = ["h%d" % i for i in range(data.shape[0])]
        vlines = np.array(
            [-1 * deltas[i] + msg.trigger_time for i in range(deltas.shape[0])],
        )
        self.debug_samples.publish_plots(plots, titles, vlines)

        try:
            vec = util.calculate_dir_pinger(deltas, self.dist_h, self.v_sound)
        except Exception as e:
            rospy.logwarn(
                "/hydrophones/ping_locator could not calculate pinger direction %s" % e,
            )
            return

        v = Vector3Stamped()
        v.header = msg.header
        v.header.frame_id = "hydrophones"
        v.vector = numpy_to_vector3(vec)
        self.pub.publish(v)


if __name__ == "__main__":
    a = PingLocator()
    rospy.spin()
