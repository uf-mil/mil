#!/usr/bin/env python
from navigator_msgs.msg import PerceptionObject, PerceptionObjects
from navigator_msgs.srv import ObjectDBSingleQueryResponse, ObjectDBFullQueryResponse, ObjectDBSingleQuery, ObjectDBFullQuery
from visualization_msgs.msg import MarkerArray, Marker
from txros import NodeHandle, util
from twisted.internet import defer, reactor

nh = None


class ObjectDatabase(object):

    def __init__(self):
        self.pose = None
        self.items = {}
        self.unknowns = {}

    @util.cancellableInlineCallbacks
    def _init(self):
        self.nh = yield NodeHandle.from_argv("my_object_database")
        global nh
        nh = self.nh

        self.pub_object_found = yield self.nh.advertise('/database/object_found', PerceptionObject)
        self.pub_object_markers = yield self.nh.advertise('/database/objects_classified', MarkerArray)
        self.pub_object_all = yield self.nh.advertise('/database/objects', PerceptionObjects)

        self.serv_single_query = yield self.nh.advertise_service('/database/single', ObjectDBSingleQuery, self.query_single)
        self.serv_full_query = yield self.nh.advertise_service('/database/full', ObjectDBFullQuery, self.query_full)

        self.sub_object_classification = yield self.nh.subscribe('/classifier/object', PerceptionObject, self.new_object)

        self.publish_items()

        defer.returnValue(self)

    def new_object(self, perception_object):
        p = perception_object
        # print p.id
        # print "----"
        # print p

        # If it is unknown, add it to the list of unknowns and exit
        if(p.type == PerceptionObject.UNKNOWN):
            if p.id not in self.unknowns:
                self.unknowns[p.id] = p
            return

        # If it isn't unknown, but used to be, remove it from the list of unknowns
        for _id in self.unknowns:
            if(p.id == _id):
                del self.unknowns[_id]
                break

        # If it has been reclassified, change the entry in the database
        for i in self.items.values():
            if i.id == p.id and i.type != p.type:
                del self.items[i.type]
                break

        # If this is a new classification, publish that a new object has been found
        if p.type not in self.items.keys():
            self.pub_object_found.publish(p)

        # Add it to the database
        self.items[p.name] = p
        print self.items

    @util.cancellableInlineCallbacks
    def publish_items(self):
        objects = PerceptionObjects()
        while True:

            for i in self.items.values():
                objects.objects.append(i)
            for i in self.unknowns.values():
                objects.objects.append(i)

            self.pub_object_all.publish(objects)
            self.add_markers()

            yield nh.sleep(.5)

    def add_markers(self):
        marker_del = Marker()
        marker_del.action = 3  # This is DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(marker_del)
        for item in self.items.values():
            print item
            marker = Marker()
            marker.header.stamp = nh.get_time()
            marker.header.seq = 1
            marker.header.frame_id = "enu"
            marker.id = item.id
            marker.pose.position = item.position
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.scale.x = 3.0
            marker.scale.y = 3.0
            marker.scale.z = 3.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.text = item.type + "_" + str(item.id)
            marker_array.markers.append(marker)

        for item in self.unknowns.values():
            marker = Marker()
            marker.header.stamp = nh.get_time()
            marker.header.seq = 1
            marker.header.frame_id = "enu"
            marker.id = item.id
            marker.pose.position = item.position
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.scale.x = 3.0
            marker.scale.y = 3.0
            marker.scale.z = 3.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.text = PerceptionObject.UNKNOWN
            marker_array.markers.append(marker)
        self.pub_object_markers.publish(marker_array)

    def query_single(self, req):
        a = req.name
        per = ObjectDBSingleQueryResponse()
        per.found = False
        if(a in self.items.keys()):
            per.object = self.items[a]
            per.found = True
        return per

    def query_full(self, req):
        per = ObjectDBFullQueryResponse()
        for item in self.items.values():
            per.all_objects.append(item)

        return per


@util.cancellableInlineCallbacks
def main():
    od = yield ObjectDatabase()._init()

reactor.callWhenRunning(main)
reactor.run()
