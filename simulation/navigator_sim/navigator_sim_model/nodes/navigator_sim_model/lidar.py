from navigator_sim_rendering import sim_rendering_helpers as srh
from navigator_sim_rendering import sim_math_helpers as smh
from OpenGL.GL import *
from OpenGL.GLU import *

class Lidar:
    def __init__(self):
        '''
        This class constitutes a lidar object. 
        
        It is packaged with:
            - Ranging "ray-tracer"
            - Laserscan publisher
            - TF publisher
        
        Future:
            To make it a more generic, useful object
                - Rotation and relative position must be settable
                - Inspiration: 'Threed' camera view
        '''

        ##TODO: This needs to be made more generic for use with sub imaging sonar
        
        self.lidar_pub = rospy.Publisher('/lidar/scan', LaserScan, queue_size=1)
        self.lidar_tf_br = tf.TransformBroadcaster()
        self.pitch = 0
        self.pitch_inc = math.radians(1)

    def pitch_lidar(self):
        '''
        pitch_lidar()
        This function pitches the lidar by a preset amount of 2 degrees per tick
        It oscillates between max_pitch and min_pitch
        '''

        max_pitch = math.radians(15)
        min_pitch = math.radians(-10)
        

        if (self.pitch < min_pitch) or (self.pitch > max_pitch):
            self.pitch_inc = -self.pitch_inc 
            #swap directions

        self.pitch += self.pitch_inc

    def send_lidar_transform(self):
        '''
        Publishes a transformation on /base_link/laser
            -> Allows LIDAR data taken at an arbitrary angle (or position) 
                to be viewed from the base_link (The boat body) reference frame.

        Reference code at laser_tf_broadcaster.py in sensors package
        (If you are looking for tf reference code, try grep -r "TransformBroadcaster"
        in the root uf-mil directory)
        '''
        ##TODO: 
        # Clean this up, share tf xyz/pitch with publish/scanner
        bl_lu_x       = 0.8001        # Distance forward, BaseLink to Lidar Unit
        bl_lu_y       = 0             # Distance left, BaseLink to Lidar Unit
        bl_lu_z       = 0.254         # Distance up, BaseLink to Lidar Unit
        lu_r          = 0             # Roll of Lidar Unit
        lu_p          = self.pitch    # Pitch of Lidar Unit
        lu_y          = 0             # Yaw of Lidar Unit

        #Transformation Matrix
        T = tf.transformations.translation_matrix((bl_lu_x, bl_lu_y, bl_lu_z))
        T = tf.transformations.rotation_matrix((lu_y), (0, 0, 1)).dot(T)
        T = tf.transformations.rotation_matrix((lu_p), (0, 1, 0)).dot(T)
        T = tf.transformations.rotation_matrix((lu_r), (1, 0, 0)).dot(T)
        T = tf.transformations.translation_from_matrix(T)

        total_roll = 0
        total_pitch = self.pitch 
        total_yaw = 0
        #So this is not a full simulation of the pivot, etc
        
        lidar_quat = tf.transformations.quaternion_from_euler(total_roll, total_pitch, total_yaw)

        self.lidar_tf_br.sendTransform(
                T,
                lidar_quat,
                rospy.Time.now(),
                "/base_link/laser",
                "/base_link",
                )

    def get_lidar_range(self):
        '''
        get_lidar_range(nothing) -> distances
        
        Behavior:
        
            This function will publish distance to the nearest object,
            in accordance with the current orientation of the dynamixel servos
            It will automatically cull the boat geom from the list of distances
        
        Functionality:
            
            Distances are computed using ode collision space, where the distance is
            the point to point distance between the origin of the ray and the point 
            on the geom that the ray intersects with
        
        Future:
            
            - Variable pitch (Sub on /lidar_angle topic)
            - Fake intensities (Based on distance, no need to go crazy)

        Lidar Simulated Features:
             
            Field of view:                        270 deg
            Scanning frequency:                   25 Hz / 50 Hz    
            Angular resolution:                   0.25 deg 0.5 deg
            Operating range:                      0.5 m to 20 m    
            Max. range with 10 % reflectivity:    18 m    
            Amount of evaluated echoes:           2 (I don't actually simulate this feature yet)
        '''
         
        lidar_FOV = 270 #deg
        lidar_scanrate = 25.0 #hz
        lidar_ang_res = 0.25 #deg
        lidar_range = 20.0
        #lidar_range = 100
        lidar_end_angle = math.radians((lidar_FOV/2.0))
        lidar_start_angle = math.radians(-(lidar_FOV/2.0))

        def publish_lidar(scan_ranges_in):
            self.lidar_pub.publish(
                LaserScan(
                    header = Header(
                        stamp=rospy.Time.now(), 
                        frame_id="/base_link/laser" 
                    ),
                    angle_min = lidar_start_angle,
                    angle_max = lidar_end_angle,
                    angle_increment = math.radians(lidar_ang_res),
                    scan_time = 1.0/lidar_scanrate,
                    range_min = 0.5,
                    range_max = lidar_range,
                    ranges = scan_ranges_in,
                )
            )

        raylength = lidar_range + 0.2*lidar_range
        #Space - ?
        ray = ode.GeomRay(None, raylength)
        # Had to do conversions, couldn't find a metric ruler
        # This vector is the physical offset of the LIDAR module from the center of the boat
        lidar_pos = v(0.8001,0.0,0.254) 

        # Pre-initialize the array
        # Rough experimentation shows that this does save time
        scan_ranges = [None]*int(lidar_FOV/lidar_ang_res)
        #xrange is now the default range function in python 3.x, if you run into porting errors, look here
        for i in xrange(len(scan_ranges)):
            
            projection_angle = math.radians(i*lidar_ang_res) - math.radians(lidar_FOV/2.0)
            
            x_hat = math.cos(projection_angle)
            y_hat = math.sin(projection_angle)
            z_hat = math.sin(self.pitch)
            
            lidar_ray_dir = body.vectorToWorld(v(x_hat, y_hat ,-z_hat))

            ray.set( body.getRelPointPos(lidar_pos), lidar_ray_dir )
            
            closest_distance = 250

            for contact in ode.collide(ray,space):
                stamp = rospy.Time.now()
                pos,normal,dist,geom1,geom2 = contact.getContactGeomParams()
                # Pos = position XYZ
                # Normal = normal vector on collided surface
                # Dist = distance to object
                # Geom1 = the ray (I think?)
                # Geom2 = the detected object geometry
                
                assert geom1 is ray, geom1
                #The purpose of the below (was) to not hit the water, but that doesn't work.
                #if (geom2 is  not lake_geom):
                #    continue
                valid_distance =  (V(pos) - V(body.getRelPointPos(lidar_pos))).mag()
                #There's no guarantee that they'll come in order, apparently.
                #Maybe play with hash space settings? Is that a things?
                if(valid_distance < closest_distance):
                    closest_distance = valid_distance

            scan_ranges[i] = closest_distance

        self.send_lidar_transform()
        publish_lidar(scan_ranges)
        self.pitch_lidar()