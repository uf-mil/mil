import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse
import cv2
from cv_bridge import CvBridge
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import matplotlib
import threading

class Plotter:
    '''Publishes several plots of 2d data over rostopic via Image msgs
    Basic Usage Example:
        #  Import
        from mil_ros_tools import Plotter
        import numpy as np
        import rospy

        #  ROS node
        rospy.init_node('my_node')

        #  Create
        my_plotter = Plotter('my_plotter_topic')

        #  Publish some plots
        titles = ['random data', 'sin wave']

        data_size = 100
        y1 = np.random.rand(data_size)
        x1 = np.arange(0, y1.shape[0])

        x2 = np.linspace(0, 5, data_size)
        y2 = np.sin(x2)

        plots = np.vstack((x1, y1, x2, y2))
        my_plotter.publish_plots(plots, titles)

    For another usage example see `mil_passive_sonar triggering`

    Limitations:
        can only stack plots vertially

        all plots must have same number of points

        cannot name axes

        Publishing happens in another thread,
            if publish_plots is called before a previous publish plots call finishes,
            the most recent publish plots call will be ignored

        cannot plot mutiple data sets on top of  each other in the same plot

        cannot change color of plots

    Features:
        Can be enables/disabled via the enable_<topic_name> service call
    '''

    def __init__(self, topic_name, w=20, h=20, dpi=200):
        matplotlib.rcParams.update({'font.size': 22})
        self.pub = rospy.Publisher(topic_name, Image, queue_size=1)
        self.bridge = CvBridge()
        self.fig = Figure(figsize=(w,h), dpi=dpi)
        self.canvas = FigureCanvasAgg(self.fig)
        self.enabled = True
        self.thread = None

        rospy.Service(('/enable_%s'%topic_name), SetBool, self.enable_disable)


    def enable_disable(self, req):
        self.enabled = req.data
        return SetBoolResponse(success=True)


    def publish_plots(self, plots, titles=[]):
        if self.pub.get_num_connections() == 0:
            return
        if not self.enabled:
            return
        if (self.thread is not None) and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self.publish_plots_, args=(plots,titles))
        self.thread.daemon = True
        self.thread.start()


    def publish_plots_(self, plots, titles=[]):

        num_of_plots = plots.shape[0]/2

        for i in xrange(1, num_of_plots+1):
            self.fig.add_subplot(num_of_plots, 1, i)
        for i, ax in enumerate(self.fig.axes):
            ax.plot(plots[i*2,:], plots[i*2+1,:])
            if i < len(titles):
                ax.set_title(titles[i])
        self.canvas.draw()

        s, (w, h) = self.canvas.print_to_buffer()

        img = np.fromstring(s, np.uint8).reshape(w, h, 4)

        img = np.roll(img, 3, axis = 2)
        for ax in self.fig.axes: ax.cla()

        # make ros msg of the img
        msg = self.bridge.cv2_to_imgmsg(img, encoding='passthrough')
        # publish the image
        self.pub.publish(msg)

