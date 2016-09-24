from navigator_msgs.msg import PerceptionObject
from navigator_msgs.srv import PerceptionObjectService, PerceptionObjectServiceResponse
import rospy
import numpy as np
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef
from visualization_msgs.msg import MarkerArray, Marker



class ObjectClassifier:
    def __init__(self):
        
        self.pose = None
        self.ecef_pose = None
        self.items = {}
        self.pub = rospy.Publisher('/vision/object_classifier', PerceptionObject, queue_size=10)
        self.serv = s = rospy.Service('/vision/object_classifier_service', PerceptionObjectService, query)
        self.pub_vis = rospy.Publisher('/rviz/objs', Marker, queue_size=10)
        self.count = 0
        self.used_positions = []
        self.objs = {}


    def query(self, name):
        if(name in self.objs.keys()):
            return PerceptionObjectServiceResponse(True, self.objs[name][0], self.objs[name][1])
        return PerceptionObjectServiceResponse(False, None, None)
        
    def set_pose(self, enu, ecef):
        self.pose = enu
        self.ecef_pose = ecef
        buoys = [29.534553,-82.303941]
        stc = [0,0]
        start_gate = [0,0]
        shooter = [0,0]
        self.items = {"buoys": self.to_lat_long(buoys), "stc": self.to_lat_long(stc),
                      "start_gate": self.to_lat_long(start_gate), "shooter": self.to_lat_long(shooter)}

                      


    def to_lat_long(self, ll, alt=0):
        lat = ll[0]
        lon = ll[1]
        ecef_pos, enu_pos = self.ecef_pose[0], self.pose[0]

        # These functions want radians
        lat, lon = np.radians([lat, lon], dtype=np.float64)
        ecef_vector = ecef_from_latlongheight(lat, lon, alt) - ecef_pos
        enu_vector = enu_from_ecef(ecef_vector, ecef_pos)
        enu_vector[2] = 0  # We don't want to move in the z at all
        return np.array(enu_pos + enu_vector)

    def classify(self, xyz, lwh):
        pos = [xyz.x,xyz.y,xyz.z]
        size = [lwh.x,lwh.y,lwh.z]
        for p in self.used_positions:
            if(np.linalg.norm(np.subtract(pos, p)) < 1):
                return
        for key,bb in self.items.items():
            if(self.in_bounding_box(pos, bb) and lwh.x*lwh.y*lwh.z > 1):
                p = PerceptionObject()
                p.name = key
                print "GOTTEM", key, pos
                if(key == 'buoys'):
                    self.add_marker_vis(pos,[0,0,1])
                if(key == 'stc'):
                    self.add_marker_vis(pos,[1,1,0])
                if(key == 'start_gate'):
                    self.add_marker_vis(pos,[0,1,1])
                if(key == 'shooter'):
                    self.add_marker_vis(pos,[1,0,1])
                p.pos = xyz
                p.size = lwh
                self.used_positions.append(pos)
                self.pub.publish(p)
                self.objs[key] = [pos,size]


    def add_marker_vis(self, pos, colors):
        marker = Marker()
        marker.header.frame_id = "enu";
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace";
        marker.id = self.count
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 3
        marker.scale.y = 3
        marker.scale.z = 3
        marker.color.a = 1.0
        marker.color.r = colors[0]
        marker.color.g = colors[1]
        marker.color.b = colors[2]
        self.pub_vis.publish(marker);
        self.count+=1




    def in_bounding_box(self, pos, bounding_box_center, radius=60):
        bbc = bounding_box_center
        r = radius/2
        bounding_box = [[bbc[0]+r, bbc[1]+r],
                        [bbc[0]+r, bbc[1]-r],
                        [bbc[0]-r, bbc[1]+r],
                        [bbc[0]-r, bbc[1]-r]]
        # self.add_marker_vis(bounding_box[0], [0,1,0])
        # self.add_marker_vis(bounding_box[1], [0,1,0])
        # self.add_marker_vis(bounding_box[2], [0,1,0])
        # self.add_marker_vis(bounding_box[3], [0,1,0])

        ab = np.subtract(bounding_box[0], bounding_box[1])
        ac = np.subtract(bounding_box[0], bounding_box[2])
        am = np.subtract(bounding_box[0], pos[:2])
        if(np.dot(ab,ac) > .1):
            ac = np.subtract(bounding_box[0], bounding_box[3])
        
        if(0 <= np.dot(ab,am) <= np.dot(ab,ab) and 0 <= np.dot(am,ac) <= np.dot(ac,ac)):
            return True

        return False