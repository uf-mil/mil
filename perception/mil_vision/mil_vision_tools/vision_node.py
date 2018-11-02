#!/usr/bin/env python
import rospy
from mil_ros_tools import Image_Subscriber, numpy_to_point2d
from mil_msgs.msg import ObjectInImage, ObjectsInImage
from std_srvs.srv import SetBool
from image_geometry import PinholeCameraModel
import numpy as np
import abc

__author__ = "Kevin Allen"


def create_object_msg(name, confidence=None, center=None, contour=None, rect=None, attributes=''):
    '''
    Helper function to create a mil_msgs/ObjectInImage message.
    Note: only set one of center, contour, or rect depending on what informaton is needed/available in your application

    @param name: string name of the identifed object
    @param attributes: Attributes string to attach to message, application specific meaning
    @param confidence: [0.0, 1.0] confidence that name is correct for this object.
                       Leave as None if confidence is not known.
    @param center: 2 wide tuple/array-like representing the center pixel of the object
    @param contour: Nx1x2 or Nx2 numpy array of pixels making up the contour around the object
    @param rect: a 4 wide tuple/array-like representing the bounding box around the object as
                 (X, Y, width, height), which is the representation returned by cv2.boundingRect
    @return: A ObjectInImage message object filled as described above
    TODO document usage
    '''
    # Create message
    msg = ObjectInImage()

    # Fill name and attributes from argument
    msg.name = name
    msg.attributes = attributes

    # Fill confidence from argument if given, otherwise use -1
    if confidence is None:
        msg.confidence = -1.
    else:
        msg.confidence = confidence

    # Fill points with contour, rect, or center depending on which is set
    if contour is not None:
        # Reshape to Nx2 incase input was given in cv's native Nx1x2 shape
        if len(contour.shape) == 3:
            contour = contour.reshape((contour.shape[0], contour.shape[2]))
        for point in contour:
            msg.points.append(numpy_to_point2d(point))
    elif rect is not None:
        # Add rectangle as upper left and bottom right points
        ul = np.array(rect[0:2])
        br = ul + np.array(rect[2:])
        msg.points.append(numpy_to_point2d(ul))
        msg.points.append(numpy_to_point2d(br))
    elif center is not None:
        # Add center tuple as single point
        msg.points.append(numpy_to_point2d(center))

    return msg


class VisionNode(object):
    '''
    Base class to be used unify the interfacing for MIL's computer vision scripts.
    Handles the bootstrap of image subscription, enable/disable, etc.
    Provides a callback for new images which is expected to return
    '''
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self._objects_pub = rospy.Publisher("~identified_objects", ObjectsInImage, queue_size=3)
        self._camera_info = None
        self.camera_model = None
        self._enabled = False
        self._image_sub = Image_Subscriber("image", callback=self._img_cb)
        if rospy.get_param("~autostart", default=False):
            self._enable()
        else:
            self._disable()
        self._enable_srv = rospy.Service("~enable", SetBool, self._enable_cb)

    def _enable_cb(self, req):
        if req.data and not self._enabled:
            self._enable()
        elif not req.data and self._enabled:
            self._disable()
        return {'success': True}

    def _enable(self):
        if self._camera_info is None:
            self._camera_info = self._image_sub.wait_for_camera_info()
            self.camera_model = PinholeCameraModel()
            self.camera_model.fromCameraInfo(self._camera_info)
        self._enabled = True
        rospy.loginfo("Enabled.")

    def _disable(self):
        self._enabled = False
        rospy.loginfo("Disabled.")

    def _img_cb(self, img):
        if not self._enabled:
            return
        msg = ObjectsInImage()
        msg.header = self._image_sub.last_image_header
        msg.objects = self.find_objects(img)
        if not isinstance(msg.objects, list) or (len(msg.objects) and not isinstance(msg.objects[0], ObjectInImage)):
            rospy.logwarn("find_objects did not return a list of mil_msgs/ObjectInImage message. Ignoring.")
        self._objects_pub.publish(msg)

    @abc.abstractmethod
    def find_objects(self, img):
        pass


if __name__ == "__main__":
    '''
    When this library is run as an executable, run a demo class
    '''
    import cv2
    from cv_tools import contour_centroid

    class VisionNodeExample(VisionNode):
        '''
        Example implementation of a VisionNode, useful only for reference in real applications
        '''
        def __init__(self):
            # Call base class's init. Important to do this if you override __init__ in child class.
            super(VisionNodeExample, self).__init__()

        def find_objects(self, img):
            # Get a list of contours in image
            blured = cv2.blur(img, (5, 5))
            edges = cv2.Canny(blured, 100, 200)
            _, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = np.array(contours)
            objects = []

            # Add each contour, randomly chosing center, contour, or rect to demonstrate all three
            # In real application, only one of the three methods will be used depending on the algorithm
            # and what information is needed.
            for idx, contour in enumerate(contours):
                # Demonstration of adding an object where only the center point can be identified
                if idx % 3 == 0:
                    try:
                        center = contour_centroid(contour)
                    except ZeroDivisionError:
                        continue
                    objects.append(create_object_msg("contour", center=center, attributes='green'))
                # Demonstration of adding an object where the entire contour outline can be identified
                if idx % 3 == 1:
                    objects.append(create_object_msg("contour", contour=contour, confidence=0.5))
                # Demonstration of adding an object where a bounding rectangle can be identified
                if idx % 3 == 2:
                    objects.append(create_object_msg("contour", rect=cv2.boundingRect(contour), confidence=0.8))

            # Log that an image has been received for debugging this demo
            rospy.loginfo("Image")
            return objects

    rospy.init_node("vision_node_example")
    node = VisionNodeExample()
    rospy.spin()
