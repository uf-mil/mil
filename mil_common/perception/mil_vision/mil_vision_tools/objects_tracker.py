#!/usr/bin/env python
import abc
import numpy as np
import rospy


__author__ = "Kevin Allen"


class TrackedObject(object):
    '''
    Contains a single object tracked by an ObjectsTracker instance.
    '''
    def __init__(self, id, stamp, features, data=None):
        '''
        Create a TrackedObject
        @param id: a unique id for this object
        @param stamp: time stamp object was observered, rospy.Time object
        @param features: feature set used by the specific implementation of ObjectsTracker
        @param data: Optional additional application-specific data to track
        '''
        self.id = id
        self.created_at = stamp
        self.observations = 0
        self.update(stamp, features, data)

    def update(self, stamp, features, data=None):
        '''
        Update an object's metadata
        @param stamp: time stamp object was just observed in
        @param featureS: feature set used by the specific implementation of ObjectsTracker
        '''
        self.data = data
        self.stamp = stamp
        self.features = features
        self.observations += 1

    @property
    def age(self):
        '''
        @return: difference in time between when the object was most recently observed to first observed
        '''
        return self.stamp - self.created_at

    def __str__(self):
        return "TrackedObject(id={}, observations={}, age={}s)".format(self.id, self.observations, self.age.to_sec())


class ObjectsTracker(object):
    '''
    Tracks an arbitrary number of objects over time based on a disance metric.
    Useful for finicky computer vision programs, ensuring an object was observed in multiple recent frames
    before using for autonomy

    Note: this is an abstract class, you must use a child class that imlements the required methods
    '''
    __metaclass__ = abc.ABCMeta

    def __init__(self, expiration_seconds=5.0, max_distance=None):
        '''
        Creates an ObjectsTracker.
        @param expiration_seconds: longest time, in seconds for old objects to be kept
        @param max_distance: float, maximum distance two objects can be away to be matched
        '''
        self.expiration_seconds = rospy.Duration(expiration_seconds)
        self.max_distance = max_distance
        self.max_id = 0
        self.objects = []

    def add_observation(self, stamp, features, data=None):
        '''
        Add a new observation to the tracked objects.
        @stamp: rospy.Time object representing the time this observation arrived
        @return: The id of the object, or -1 if it was added
        '''
        for obj in self.objects:
            if self.distance(obj.features, features) < self.max_distance:
                obj.update(stamp, features, data=data)
                return obj

        # No match found, add new
        new_obj = TrackedObject(self.max_id, stamp, features, data=data)
        self.max_id += 1
        self.objects.append(new_obj)
        return new_obj

    def clear_expired(self, now=None):
        '''
        Deletes expired objects
        Should be called frequently to clear expired objets
        '''
        if now is None:
            now = rospy.Time.now()
        self.objects = filter(lambda obj: now - obj.stamp < self.expiration_seconds, self.objects)

    def get_persistent_objects(self, min_observations=10, min_age=rospy.Duration(0)):
        '''
        Get a list of objects which have persisted sufficiently long.
        @param min_observations: Minimum number of times the object was seen to be returned
        @param min_age: Minimum age an object must be to be returned in result
        @return: List of TrackedObject instances of those objects meeting the above criteria
        '''
        return filter(lambda obj: obj.age >= min_age and obj.observations >= min_observations, self.objects)

    @abc.abstractmethod
    def distance(self, a, b):
        pass


class CentroidObjectsTracker(ObjectsTracker):
    '''
    Implements ObjectsTracker, using the distance between centroids of observations to track.

    Features must be added as a (2,) numpy array (Cx, Cy)
    '''
    def __init__(self, max_distance=10.0, **kwargs):
        super(CentroidObjectsTracker, self).__init__(max_distance=max_distance, **kwargs)

    def distance(self, a, b):
        '''
        Calculates distance by the euclidian distance between the centroids
        '''
        return np.linalg.norm(a - b)
