import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
import cv2
from cv_bridge import CvBridge
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import matplotlib
import threading


class Plotter:
    """
    Publishes several plots of 2D data over rostopic via :class:`Image` messages.

    Basic Usage Example:

    .. code-block:: python3

        >>> #  Import
        >>> from mil_ros_tools import Plotter
        >>> import numpy as np
        >>> import rospy
        >>> #  ROS node
        >>> rospy.init_node('my_node')
        >>> #  Create
        >>> my_plotter = Plotter('my_plotter_topic')
        >>> #  Publish some plots
        >>> titles = ['random data', 'sin wave']
        >>> data_size = 100
        >>> y1 = np.random.rand(data_size)
        >>> x1 = np.arange(0, y1.shape[0])
        >>> x2 = np.linspace(0, 5, data_size)
        >>> y2 = np.sin(x2)
        >>> plots = np.vstack((x1, y1, x2, y2))
        >>> my_plotter.publish_plots(plots, titles)

    For another usage example see `mil_passive_sonar triggering`.

    Attributes:
        pub (rospy.Publisher): The ROS publisher node. The topic name is passed
            in upon construction and the queue size is 1.
        thread (threading.Thread): The thread used to publish data and plots on.
    """
    # Limitations:
    #     can only stack plots vertially
    #     all plots must have same number of points
    #     cannot name axes
    #     Publishing happens in another thread,
    #         if publish_plots is called before a previous publish plots call finishes,
    #         the most recent publish plots call will be ignored
    #     cannot plot mutiple data sets on top of  each other in the same plot
    #     cannot change color of plots
    # Features:
    #     Can be enables/disabled via the <topic_name>_enable service call

    def __init__(self, topic_name: str, w: int = 20, h: int = 20, dpi: int = 150):
        matplotlib.rcParams.update({"font.size": 22})
        self.pub = rospy.Publisher(topic_name, Image, queue_size=1)
        self.bridge = CvBridge()
        self.fig = Figure(figsize=(w, h), dpi=dpi)
        self.canvas = FigureCanvasAgg(self.fig)
        self.enabled = True
        self.thread = None

        rospy.Service(("%s_enable" % topic_name), SetBool, self.enable_disable)

    def enable_disable(self, req: SetBoolRequest):
        """
        Serves as a callback for the service responsible for enabling and disabling the 
        plotter.

        Args:
            req (SetBoolRequest): The message received by the service.

        Returns:
            SetBoolResponse: The response sent back by the service.
        """
        self.enabled = req.data
        return SetBoolResponse(success=True)

    def is_go(self) -> bool:
        """
        Whether to run the plotter. ``True`` if the plotter is enabled, there is
        more than one connection connected to the publisher and the thread is ``None``.

        Returns:
            bool: Whether to run the plotter.
        """
        return (
            self.enabled
            and self.pub.get_num_connections() > 0
            and (self.thread is None or not self.thread.is_alive())
        )

    def publish_plots(self, plots, titles=[], v_lines=[]):
        """
        Starts as new thread to publish the data on a plot.
        """
        if self.is_go():
            self.thread = threading.Thread(
                target=self.publish_plots_, args=(plots, titles, v_lines)
            )
            self.thread.daemon = True
            self.thread.start()

    def publish_plots_(self, plots, titles=[], v_lines=[]):

        num_of_plots = plots.shape[0] / 2

        for i in range(1, num_of_plots + 1):
            self.fig.add_subplot(num_of_plots, 1, i)
        for i, ax in enumerate(self.fig.axes):
            ax.plot(plots[i * 2, :], plots[i * 2 + 1, :])
            if i < len(titles):
                ax.set_title(titles[i])
            if i < len(v_lines):
                ax.axvline(v_lines[i])
        self.canvas.draw()

        s, (w, h) = self.canvas.print_to_buffer()
        self.fig.clf()
        img = np.fromstring(s, np.uint8).reshape(w, h, 4)

        img = np.roll(img, 3, axis=2)
        for ax in self.fig.axes:
            ax.cla()

        # make ros msg of the img
        msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        # publish the image
        self.pub.publish(msg)


def interweave(x: np.ndarray, data: np.ndarray):
    """
    Helper function of place a single x axis in every other row of a data matrix.

    Args:
        x (np.ndarray): An array of shape (samples,)
        data (np.ndarray): An array of shape (channels, samples)

    Returns:
        np.ndarray: Array of shape (channles*2, samples) where even numbered 
            rows are x, odd rows are the data.
    """
    plots = [None] * data.shape[0]
    for i in range(data.shape[0]):
        plots[i] = np.vstack((x, data[i, :]))
    plots = np.vstack(tuple(plots))
    return plots
