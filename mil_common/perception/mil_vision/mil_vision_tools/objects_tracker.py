#!/usr/bin/env python3
import abc
import numpy as np
import rospy

from typing import List, Any, Optional


__author__ = "Kevin Allen"


class TrackedObject:
    """
    Contains a single object tracked by an :class:`ObjectsTracker` instance.

    Attributes:
        id (int): The ID of the tracked object.
        created_at (rospy.Time): A timestamp of when the object was observed.
        stamp (rospy.Time): A timestamp of when the object was most recently observed.
        features (List[Any]): A list of features used by :class:`ObjectsTracker`.
        observations (int): A number of observations executed on the object.
    """
    def __init__(self, id: int, stamp: rospy.Time, features: List[Any], data: Optional[Any] = None):
        """
        Args:
            id (int): A unique id for this object.
            stamp (rospy.Time): Time stamp object was observered.
            features (List[Any]): Feature set used by the specific implementation
                of ObjectsTracker.
            data (Optional[Any]): Optional additional application-specific data to track
        """
        self.id = id
        self.created_at = stamp
        self.observations = 0
        self.update(stamp, features, data)

    def update(self, stamp: rospy.Time, features: List[Any], data: Optional[Any] = None) -> None:
        """
        Update an object's metadata.

        Args:
            stamp (rospy.Time): Time stamp object was just observed in.
            features (List[Any]): Feature set used by the specific implementation
                of ObjectsTracker.
        """
        self.data = data
        self.stamp = stamp
        self.features = features
        self.observations += 1

    @property
    def age(self):
        """
        Difference in time between when the object was most recently observed
        compared to when it was first observed.

        Returns:
            genpy.Duration: The difference in the times.
        """
        return self.stamp - self.created_at

    def __str__(self):
        return "TrackedObject(id={}, observations={}, age={}s)".format(
            self.id, self.observations, self.age.to_sec()
        )


class ObjectsTracker:
    """
    Tracks an arbitrary number of objects over time based on a disance metric.
    Useful for finicky computer vision programs, ensuring an object was observed
    in multiple recent frames before using for autonomy.

    This is an abstract class, meaning that you must use a child class that
    implements the required methods.

    Attributes:
        expiration_seconds (rospy.Duration): A duration object representing the maximum
            time that old objects can be kept before being removed.
        max_distance (float): THe maximum distance two objects can be away to be matched.
        objects (List[TrackedObject]): A list of objects being tracked.
        max_id (int): The maximum ID being used amongst the objects. This ID is used
            when constructing new objects, after which, it is incremented by one.
    """
    __metaclass__ = abc.ABCMeta

    objects: List[TrackedObject]

    def __init__(self, expiration_seconds: int = 5, max_distance: float = 0):
        """
        Args:
            expiration_seconds (int): Longest time, in seconds, for old objects
                to be kept.
            max_distance (float): Maximum distance two objects can be away to
                be matched.
        """
        self.expiration_seconds = rospy.Duration(expiration_seconds)
        self.max_distance = max_distance
        self.max_id = 0
        self.objects = []

    def add_observation(self, stamp: rospy.Time, features: List[Any], data: Optional[Any] = None) -> TrackedObject:
        """
        Add a new observation to the tracked objects.

        Args:
            stamp (rospy.Time): Time object representing the time this observation
                arrived.
            features (List[Any]): A list of features to find the distance of, relative
                to the features stored by each object.
            data (Optional[Any]): Optional data to store with each object. Defaults to
                ``None``.

        Returns:
            TrackedObject: The object that was first updated, or the new object
            that was just constructed.
        """
        for obj in self.objects:
            if self.distance(obj.features, features) < self.max_distance:
                obj.update(stamp, features, data=data)
                return obj

        # No match found, add new
        new_obj = TrackedObject(self.max_id, stamp, features, data=data)
        self.max_id += 1
        self.objects.append(new_obj)
        return new_obj

    def clear_expired(self, now: Optional[rospy.Time] = None):
        """
        Deletes expired objects. Should be called frequently to clear expired objets.

        Args:
            now (rospy.Time): The time to compare to each object's timestamp to see
                if it has expired and needs to be removed from :attr:`.objects`.
        """
        if now is None:
            now = rospy.Time.now()
        self.objects = list(filter(
            lambda obj: now - obj.stamp < self.expiration_seconds, self.objects
        ))

    def get_persistent_objects(self, min_observations: int = 10, min_age: rospy.Duration = rospy.Duration(0)):
        """
        Get a list of objects which have persisted sufficiently long.

        Args:
            min_observations (int): Minimum number of times the object was seen to
                be returned. Defaults to 10.
            min_age (rospy.Duration): Minimum age an object must be to be
                returned in result. Defaults to a duration of zero.

        Returns:
            List[TrackedObject]: List of tracked objects meeting the above criteria.
        """
        return list(filter(
            lambda obj: obj.age >= min_age and obj.observations >= min_observations,
            self.objects,
        ))

    @abc.abstractmethod
    def distance(self, a: Any, b: Any) -> float:
        pass


class CentroidObjectsTracker(ObjectsTracker):
    """
    Implements ObjectsTracker, using the distance between centroids of observations to track.

    Features must be added as a (2,) numpy array (Cx, Cy)
    """

    def __init__(self, max_distance=10.0, **kwargs):
        super(CentroidObjectsTracker, self).__init__(
            max_distance=max_distance, **kwargs
        )

    def distance(self, a, b):
        """
        Calculates distance by the euclidian distance between the centroids
        """
        return np.linalg.norm(a - b)
