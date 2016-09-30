from navigator_msgs.msg import PerceptionObject
from navigator_msgs.srv import PerceptionObjectService, PerceptionObjectServiceResponse
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3, Pose
from navigator_tools import rosmsg_to_numpy, odometry_to_numpy
from txros import NodeHandle, util
from twisted.internet import defer

nh = None

class VisualObject:
    def __init__(self, name, size):
        self.name = name
        self.size = size
        self.last_viewed = nh.get_time()
        self.position = [100000,100000,100000]
        self.found_before = False
        self.odom = None

    def check_object(self, new_pos, odom):
        # print self.name, "found"
        self.found_before = True
        self.last_viewed = nh.get_time()
        self.position = new_pos
        self.odom = odom
        return True

class VisualObjectCollection:
    def __init__(self, name, size):
        self.name = name
        self.size = size
        self.objects  = []
        self.found_before = False
        self.odom = None

    def add_object(self, pos, odom):
        o = VisualObject(self.name, self.size)
        o.position = pos
        o.found_before = True
        o.odom = odom
        self.odom = odom
        self.objects.append(o)

    def check_object(self, new_pos, odom):
        # print self.name, "found"
        self.found_before = True
        for i, obj in enumerate(self.objects):
            if(np.linalg.norm(np.subtract(obj.position, new_pos)) < obj.last_viewed.to_sec() * .25):
                self.objects[i].last_view = nh.get_time()
                self.objects[i].position = new_pos
                self.objects[i].odom = odom
                self.odom = odom
                return True
        return False 



class ObjectClassifier:

    def __init__(self):
        print "init OC"
        self.pose = None
        # TODO: USE A YAML
        

    @util.cancellableInlineCallbacks
    def _init(self, _nh):
        print "init node"
        self.nh = _nh
        global nh
        nh = self.nh
        self.pub = yield self.nh.advertise('object_classifier', PerceptionObject)
        self.serv = yield self.nh.advertise_service('/vision/object_classifier_service', PerceptionObjectService, self.query)
        self.odom_sub = yield self.nh.subscribe('/odom', Odometry, self.get_odom)
        self.pub_vis = yield self.nh.advertise('/rviz/objs', Marker)
        
        print "published"
        self.items = {}
        self.items["buoy_field"] = VisualObjectCollection("buoy_field", [1,1,1])
        self.items["shooter"] = VisualObject("shooter", [1.8,0,1.2])
        self.items["scan_the_code"] = VisualObject("scan_the_code", [.3,0,.4])
        self.count = 0
        defer.returnValue(self)

    def get_odom(self, odom):
        self.pose = odometry_to_numpy(odom)[0]


    def query(self, req):
        name = req.name
        print self.items
        print type(name)
        print self.items[name]
        if(name not in self.items.keys() or not self.items[name].found_before):
            return PerceptionObjectServiceResponse(False, False, None, None, None)

        myobj = self.items[name]

        xyz_arr = self.pose[0]

        if(name is 'buoy_field'):
            myobj = min(myobj.objects, key=lambda y : np.linalg.norm(np.subtract(xyz_arr,y.position)))

        pos = Vector3()
        pos.x = myobj.position[0]
        pos.y = myobj.position[1]
        pos.z = 0

        size = Vector3()
        size.x = myobj.size[0]
        size.y = myobj.size[1]
        size.z = myobj.size[2]

        #uncertainty = myobj.last_view.to_sec()

        mypose = myobj.odom
        pose = Pose()
        pose.position.x = mypose[0][0]
        pose.position.y = mypose[0][1]
        pose.position.z = mypose[0][2]
        pose.orientation.x = mypose[1][0]
        pose.orientation.y = mypose[1][1]
        pose.orientation.z = mypose[1][2]
        pose.orientation.w = mypose[1][3]
        print pose

        found = False
        if(nh.get_time() - myobj.last_viewed > rospy.Duration(.5)):
            found = True

        return PerceptionObjectServiceResponse(found, True, pos, size, pose)
       
    def classify(self, xyz, lwh):
        pos = [xyz.x,xyz.y,xyz.z]
        size = [lwh.x,lwh.y,lwh.z]
        for name, obj in self.items.iteritems():
            size_item = obj.size
            errx = .5
            erry = .5
            errz = .5
            if(size[0] > size_item[0] - errx and size[0] < size_item[0] + errx and
               size[1] > size_item[1] - erry and size[1] < size_item[1] + erry and 
               size[2] > size_item[2] - errz and size[2] < size_item[2] + errz):
                myodom = self.pose
                # if(not obj.found):
                    
                if(name == 'buoy_field'):
                    self.add_marker_vis(pos,[0,0,1])
                if(name == 'scan_the_code'):
                    self.add_marker_vis(pos,[1,1,0])
                if(name == 'shooter'):
                    self.add_marker_vis(pos,[1,0,1])
                    print size
                    print "GOTTEM", name, obj
                    print self.count
                if(not obj.check_object(pos, myodom)):
                    obj.add_object(pos, myodom)
                p = PerceptionObject()
                p.name = name
                #p.odom = myodom
                p.pos = xyz
                p.size = lwh
                self.pub.publish(p)

            
    def add_marker_vis(self, pos, colors):
        marker = Marker()
        marker.header.frame_id = "enu";
        marker.header.stamp = nh.get_time()
        marker.ns = "perception_objects";
        import random
        marker.id = random.randint(500,10000000)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 1.5
        marker.color.a = 1.0
        marker.color.r = colors[0]
        marker.color.g = colors[1]
        marker.color.b = colors[2]
        self.pub_vis.publish(marker);
        self.count+=1

