"""Use the DBHelper class to interface with the Database without having to deal with ROS things."""
from navigator_msgs.msg import PerceptionObjectArray
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from twisted.internet import defer
from txros import util
from sets import Set
__author__ = "Tess Bianchi"


class DBHelper(object):
    """DBHelper class."""

    def __init__(self, nh):
        """Initialize the DB helper class."""
        self.found = Set()
        self.nh = nh
        self.new_object_subscriber = None

    @util.cancellableInlineCallbacks
    def init_(self):
        """Initialize the txros parts of the DBHelper."""
        self._sub_database = yield self.nh.subscribe('/database/objects', PerceptionObjectArray, self.object_cb)
        self._database = yield self.nh.get_service_client("/database/requests", ObjectDBQuery)

        defer.returnValue(self)

    @util.cancellableInlineCallbacks
    def begin_observing(self, cb):
        """
        Get notified when a new object is added to the database.

        cb : The callback that is called when a new object is added to the database
        """
        self.new_object_subscriber = cb
        req = ObjectDBQueryRequest()
        req.name = 'all'
        resp = yield self._database(req)

        for o in resp.objects:
            # The is upper call is because the first case is upper case if it is a 'fake' object... WHYYYYY
            if o.name not in self.found and not o.name[0].isupper():
                self.found.add(o.name)
                self.new_object_subscriber(o)

    def object_cb(self, perception_objects):
        """Callback for the object database."""
        for o in perception_objects.objects:
            if o.name not in self.found and not o[0].isupper():
                self.found.add(o.name)
                if self.new_object_subscriber is not None:
                    self.new_object_subscriber(o)

    def set_color(self, color, name):
        """Set the color of an object in the database."""
        raise NotImplementedError()

    def get_object(self, name):
        """Get an object from the database."""
        raise NotImplementedError()

    def set_fake_position(self, pos):
        """Set the position of a fake perception object."""
        raise NotImplementedError()
