"""Use the DBHelper class to interface with the Database without having to deal with ROS things."""
from navigator_msgs.msg import PerceptionObjectArray
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from nav_msgs.msg import Odometry
from twisted.internet import defer, threads
from txros import util
import time
import sys
from sets import Set
from missing_perception_object import MissingPerceptionObject
import navigator_tools as nt
import numpy as np
__author__ = "Tess Bianchi"


class DBHelper(object):
    """DBHelper class."""

    def __init__(self, nh):
        """Initialize the DB helper class."""
        self.found = Set()
        self.nh = nh
        self.position = None
        self.rot = None
        self.new_object_subscriber = None
        self.ensuring_objects = False
        self.ensuring_object_dep = None
        self.ensuring_object_cb = None
        self.looking_for = None
        self.is_found = False

    @util.cancellableInlineCallbacks
    def init_(self, navigator=None):
        """Initialize the txros parts of the DBHelper."""
        self._sub_database = yield self.nh.subscribe('/database/objects', PerceptionObjectArray, self.object_cb)
        self._database = yield self.nh.get_service_client("/database/requests", ObjectDBQuery)
        self.navigator = navigator
        if navigator is None:
            self._odom_sub = yield self.nh.subscribe('/odom', Odometry, self._odom_cb)
        else:
            self.position = yield navigator.tx_pose
            self.position = self.position[0]
        defer.returnValue(self)

    def _odom_cb(self, odom):
        self.position, self.rot = nt.odometry_to_numpy(odom)[0]

    @util.cancellableInlineCallbacks
    def get_object_by_id(self, my_id):
        req = ObjectDBQueryRequest()
        req.name = 'all'
        resp = yield self._database(req)
        ans = [obj for obj in resp.objects if obj.id == my_id][0]
        defer.returnValue(ans)

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
        req.name = 'All'
        resp1 = yield self._database(req)
        for o in resp.objects:
            # The is upper call is because the first case is upper case if it is a 'fake' object... WHYYYYY
            if o.name not in self.found:
                self.found.add(o.name)
                self.new_object_subscriber(o)

        for o in resp1.objects:
            # The is upper call is because the first case is upper case if it is a 'fake' object... WHYYYYY
            if o.name not in self.found:
                self.found.add(o.name)
                self.new_object_subscriber(o)

    @util.cancellableInlineCallbacks
    def get_unknown_and_low_conf(self):
        req = ObjectDBQueryRequest()
        req.name = 'all'
        resp = yield self._database(req)
        m = []
        for o in resp.objects:
            if o.name == 'unknown':
                m.append(o)
            elif o.confidence < 50:
                m.append(o)
        defer.returnValue(m)

    def set_looking_for(self, name):
        self.looking_for = name

    def found(self):
        if self.is_found:
            self.looking_for = None
            self.is_found = False
            return True
        return False

    def object_cb(self, perception_objects):
        """Callback for the object database."""
        self.total_num = len(perception_objects.objects)
        for o in perception_objects.objects:
            if o.name == self.looking_for:
                self.is_found = True
            if o.name not in self.found:
                self.found.add(o.name)
                if self.new_object_subscriber is not None:
                    self.new_object_subscriber(o)
        if self.ensuring_objects:
            missings_objs = []
            names = (o.name for o in perception_objects.objects)
            for o in self.ensuring_object_dep:
                if o not in names:
                    missings_objs.append(o)
            if len(missings_objs) > 0:
                self.ensuring_object_cb(missings_objs)

    def remove_found(self, name):
        """Remove an object that has been listed as found."""
        self.found.remove(name)

    def ensure_object_permanence(self, object_dep, cb):
        """Ensure that all the objects in the object_dep list remain in the database. Call the callback if this isn't true."""
        if object_dep is None or cb is None:
            return
        self.ensuring_objects = True
        self.ensuring_object_cb = cb
        self.ensuring_object_dep = object_dep

    def stop_ensuring_object_permanence(self):
        """Stop ensuring that objects remain in the database."""
        self.ensuring_objects = False

    def _wait_for_position(self, timeout=10):
        count = 0
        while self.position is None:
            if self.navigator is not None:
                self.position = self.navigator.pose[0]
            if count > timeout:
                return False
            count += 1
            time.sleep(1)
        return True

    @util.cancellableInlineCallbacks
    def get_closest_object(self, objects):
        """Get the closest mission."""
        pobjs = []
        for obj in objects:
            req = ObjectDBQueryRequest()
            req.name = obj.name
            resp = yield self._database(req)
            if len(resp.objects) != 0:
                pobjs.extend(resp.objects)

        if len(pobjs) == 0:
            raise MissingPerceptionObject("All")

        min_dist = sys.maxint
        min_obj = None
        for o in pobjs:
            dist = yield self._dist(o)
            if dist < min_dist:
                min_dist = dist
                min_obj = o

        defer.returnValue(min_obj)

    @util.cancellableInlineCallbacks
    def _dist(self, x):
        if self.position is None:
            success = yield threads.deferToThread(self._wait_for_position)
            if not success:
                raise Exception("There is a problem with odom.")
        defer.returnValue(np.linalg.norm(nt.rosmsg_to_numpy(x.position) - self.position))

    @util.cancellableInlineCallbacks
    def get_object(self, object_name, volume_only=False, thresh=30, thresh_strict=10):
        """Get an object from the database."""
        if volume_only:
            req = ObjectDBQueryRequest()
            req.name = object_name
            resp = yield self._database(req)
            if not resp.found:
                raise MissingPerceptionObject(object_name)
            defer.returnValue(resp.objects)
        else:
            req = ObjectDBQueryRequest()
            req.name = "all"
            resp = yield self._database(req)
            closest_potential_object = None
            min_dist = sys.maxint
            actual_objects = []
            for o in resp.objects:
                distance = self._dist(o)
                if o.name == object_name and distance < thresh_strict:
                    actual_objects.append(o)
                if distance < thresh and distance < min_dist:
                    min_dist = distance
                    closest_potential_object = o
            # print [x.name for x in actual_objects]
            # print closest_potential_object.name
            # sys.exit()

            if len(actual_objects) == 0 and min_dist == sys.maxint:
                raise MissingPerceptionObject(object_name)

            if len(actual_objects) > 1:
                min_dist = sys.maxint
                min_obj = None
                for o in actual_objects:
                    dist = yield self._dist(o)
                    if dist < min_dist:
                        min_dist = dist
                        min_obj = o
                defer.returnValue(min_obj)

            if len(actual_objects) == 1:
                defer.returnValue(actual_objects[0])

            defer.returnValue(closest_potential_object)

    def wait_for_additional_objects(self, timeout=60):
        num_items = self.num_items
        start = time()
        while(timeout < time() - start):
            if self.num_items > num_items:
                return True
        return False

    def set_color(self, color, name):
        """Set the color of an object in the database."""
        raise NotImplementedError()

    def set_fake_position(self, pos):
        """Set the position of a fake perception object."""
        raise NotImplementedError()
