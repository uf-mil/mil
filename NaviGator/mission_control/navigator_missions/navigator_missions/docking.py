#!/usr/bin/env python3
import copy
import random
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from dynamic_reconfigure.msg import DoubleParameter
from geometry_msgs.msg import Pose
from image_geometry import PinholeCameraModel
from mil_tools import pose_to_numpy, rosmsg_to_numpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool, SetBoolRequest
from tf.transformations import quaternion_matrix

from .navigator import NaviGatorMission
from navigator_vision import GripPipeline

PANNEL_MAX = 0
PANNEL_MIN = 2

CAMERA_LINK_OPTICAL = "wamv/front_left_camera_link_optical"

COLOR_SEQUENCE_SERVICE = "/vrx/scan_dock/color_sequence"

TIMEOUT_SECONDS = 30


class Docking(NaviGatorMission):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ogrid = None
        self.ogrid_origin = None
        self.ogrid_cpm = None
        self.ogrid_sub = self.nh.subscribe(
            "/ogrid",
            OccupancyGrid,
            callback=self.ogrid_cb,
        )

        # Service to save and restore the settings of PCODAR
        self.pcodar_save = self.nh.get_service_client("/pcodar/save", SetBool)
        # self.image_sub = self.nh.subscribe(
        #     "/wamv/sensors/cameras/front_left_camera/image_raw", Image
        # )
        self.image_sub = self.nh.subscribe(
            "/wamv/sensors/camera/front_left_cam/image_raw",
            Image,
        )
        self.cam_frame = None
        # self.image_info_sub = self.nh.subscribe(
        #     "/wamv/sensors/cameras/front_left_camera/camera_info", CameraInfo
        # )
        self.image_info_sub = self.nh.subscribe(
            "/wamv/sensors/camera/front_left_cam/camera_info",
            CameraInfo,
        )
        self.model = PinholeCameraModel()
        self.center = None
        self.intup = lambda arr: tuple(np.array(arr, dtype=np.int64))
        self.last_image = None

        self.contour_pub = self.nh.advertise("/contour_pub", Image)

    @classmethod
    async def init(cls):
        pass

    @classmethod
    async def shutdown(cls):
        await cls.contour_pub.shutdown()
        await cls.ogrid_sub.shutdown()
        await cls.image_sub.shutdown()
        await cls.image_info_sub.shutdown()

    async def run(self, args):
        await self.contour_pub.setup()
        rospy.logerr("RUN START")
        await self.ogrid_sub.setup()
        rospy.logerr("OGRID DONE")
        await self.image_sub.setup()
        rospy.logerr("IMAGE DONE")
        await self.image_info_sub.setup()
        rospy.logerr("INFO DONE")
        await self.pcodar_save.wait_for_service()
        rospy.logerr("PCODAR DONE")
        
        self.grip = GripPipeline()
        await self.change_wrench("autonomous")

        self.bridge = CvBridge()
        msg = await self.image_info_sub.get_next_message()
        self.model.fromCameraInfo(msg)
        rospy.logerr("HERE")

        self.cam_frame = (await self.image_sub.get_next_message()).header.frame_id
        rospy.logerr("HERE2")

        # Save PCODAR settings
        await self.pcodar_save(SetBoolRequest(True))

        # Change cluster tolerance to make it easier to find dock
        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 10
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        rospy.logerr("HERE3")

        # Get the POI (point of interest) from the config file and move to it
        # This is a predetermined position of the general location for the dock
        pos = await self.poi.get("dock")
        rospy.logerr("HERE4")
        pos = pos[0]
        await self.move.set_position(pos).look_at(pos).go()

        # Decrease cluster tolerance as we approach dock since lidar points are more dense
        # This helps scenario where stc buoy is really close to dock
        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 4
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        await self.nh.sleep(5)

        # move to the open side of the dock
        await self.move_to_correct_side()

        # retry calculation to make sure we really found the open side
        await self.move_to_correct_side()

        # Temp goal color until we can get the actual color that we want to dock to
        goal_color = "Red"
        correct_dock_number = -1

        # The LIDAR to camera mapping is very unreliable, so we loop until it is correct
        while True:
            # get the dock object from the database
            dock, pos = await self.get_sorted_objects(name="dock", n=1)

            # dock is PerceptionObject
            position, rotation, dock = self.get_dock_data(dock)

            # get LIDAR points
            points = rosmsg_to_numpy(dock.points)

            # get transform from enu to boat space
            enu_to_boat = await self.tf_listener.get_transform("wamv/base_link", "enu")
            corrected = np.empty(points.shape)

            # convert LIDAR points from enu to boat space
            for i in range(points.shape[0]):
                corrected[i] = enu_to_boat.transform_point(points[i])

            # get the centers and clusters that represent the backside of the dock
            centers, clusters = self.get_cluster_centers(corrected)

            # Sort both the centers and the clusters by y coordinate
            sorted_indices = centers[:, 1].argsort()[::-1]
            centers = centers[sorted_indices]
            clusters = [clusters[i] for i in sorted_indices]

            # crop the images to get bbox and find color
            images = await self.crop_images(clusters, centers)
            
            # preferred height (x) for cropped image: 170-180
            # preferred width (y) for cropped image: 130-150

            # OLD USER BASED METHOD OF FINDING DOCK COLOR
            # When 'e' key is pressed, the current set of images will be chosen and move forward
            # When another key is pressed, we try again
            # if images[0].size > 0:
            #     cv2.imshow('Image 0 (press e if all images are correct) ', cv2.cvtColor(images[0], cv2.COLOR_BGR2RGB))
            # if images[1].size > 0:
            #     cv2.imshow('Image 1 (press e if all images are correct) ', cv2.cvtColor(images[1], cv2.COLOR_BGR2RGB))
            # if images[2].size > 0:
            #     cv2.imshow('Image 2 (press e if all images are correct) ', cv2.cvtColor(images[2], cv2.COLOR_BGR2RGB))
            # key = cv2.waitKey(0) & 0xFF
            # if key == ord('e'):
            #     cv2.destroyAllWindows()
            #     break
            # cv2.destroyAllWindows()  # Close the image window before re-looping

            # If the images are correct, then break. We need to check if the height ranges from 170-180 and width ranges from 130-150
            if images[0].shape[0] in range(90, 191) and images[0].shape[1] in range(101, 180) and images[1].shape[0] in range(90, 191) and images[1].shape[1] in range(101, 180) and images[2].shape[0] in range(90, 191) and images[2].shape[1] in range(101, 180):
                correct_dock_number = self.find_color(images, goal_color)
                if correct_dock_number != -1:
                    break

        rospy.logerr(f"Here is the correct dock number: {correct_dock_number}")

        # temporary code that just moves boat to center of cluster with whatever color was specified
        left = copy.deepcopy(centers[correct_dock_number])

        # calculate center of cluster and move towards it but at an offset distance
        left[0] = 0
        forward = copy.deepcopy(centers[correct_dock_number])
        forward[0] = forward[0] - 5
        boat_to_enu = await self.tf_listener.get_transform("enu", "wamv/base_link")
        centers[correct_dock_number] = boat_to_enu.transform_point(left)
        nextPt = boat_to_enu.transform_point(forward)
        await self.move.set_position(centers[correct_dock_number]).go(blind=True, move_type="skid")
        await self.move.set_position(nextPt).go(blind=True, move_type="skid")

        # Align with hole -> work in progress, see navigator_vision/dockdeliver_pipeline.py in navigator vision
        #image = await self.image_sub.get_next_message()
        #image = self.bridge.imgmsg_to_cv2(image)   
        #self.grip.process(image)

        
        # Shoot racquet ball projectile
        rospy.logerr("- BEFORE SHOOT PROJ -")
        if correct_dock_number != -1 and correct_dock_number is not None:
            await self.shoot_projectile(images[correct_dock_number])
        await self.pcodar_save(SetBoolRequest(False))

        await self.contour_pub.shutdown()
        await self.ogrid_sub.shutdown()
        await self.image_sub.shutdown()
        await self.image_info_sub.shutdown()
        await self.pcodar_save.shutdown()

    def get_dock_data(self, dock):
        dock = dock[0]
        position, quat = pose_to_numpy(dock.pose)
        rotation = quaternion_matrix(quat)
        return position, rotation, dock

    async def move_to_correct_side(self):
        await self.find_dock()

        # get a vector to the longer side of the dock
        dock, pos = None, None
        while dock is None or pos is None:
            try:
                # looks the the LIDAR cluster database and finds the object with name "dock"
                dock, pos = await self.get_sorted_objects(name="dock", n=1)
            except Exception as _:
                # retries if an exception occurs
                await self.find_dock()
                dock, pos = await self.get_sorted_objects(name="dock", n=1)

        # find the open side of the dock
        side, position = self.get_correct_side(dock)

        # move the boat to the middle of the correct side offset by 2 meters back and face the center of the dock
        await self.move.set_position(side).look_at(position).backward(2).go()

    def get_correct_side(self, dock):
        # dock is PerceptionObject
        position, rotation, dock = self.get_dock_data(dock)

        # get bounding box of the dock
        bbox_large = rosmsg_to_numpy(dock.scale)
        bbox_copy = copy.deepcopy(bbox_large)

        bbox_large[2] = 0
        bbox_small = copy.deepcopy(bbox_large)

        max_dim = np.argmax(bbox_large[:2])
        bbox_large[max_dim] = 0
        bbox_enu = np.dot(rotation[:3, :3], bbox_large)

        min_dim = np.argmin(bbox_small[:2])
        bbox_small[min_dim] = 0
        bbox_enu_small = np.dot(rotation[:3, :3], bbox_small)
        # this black magic uses the property that a rotation matrix is just a
        # rotated cartesian frame and only gets the vector that points towards
        # the longest side since the vector pointing that way will be at the
        # same index as the scale for the smaller side. This is genius!
        # - Andrew Knee

        # move to first attempt
        print("moving in front of dock")
        # curr_pose = await self.tx_pose()
        side_a = bbox_enu + position
        side_b = -bbox_enu + position
        side_c = bbox_enu_small + position
        side_d = -bbox_enu_small + position

        return (
            self.calculate_correct_side(
                copy.deepcopy(side_a),
                copy.deepcopy(side_b),
                copy.deepcopy(side_c),
                copy.deepcopy(side_d),
                position,
                rotation[:3, :3],
                bbox_copy,
            ),
            position,
        )

    # separates the clusters using k means clustering
    def get_cluster_centers(self, data):

        # cut off all points below the mean z value
        mean = np.mean(data, axis=0)[2]
        data = data[data[:, 2] > mean]
        centroids = []

        # Sample initial centroids
        random_indices = random.sample(range(data.shape[0]), 3)
        for i in random_indices:
            centroids.append(data[i])

        # Create a list to store which centroid is assigned to each dataset
        assigned_centroids = [0] * len(data)

        def compute_l2_distance(x, centroid):
            # Initialise the distance to 0
            dist = 0

            # Loop over the dimensions. Take squared difference and add to dist

            for i in range(len(x)):
                dist += (centroid[i] - x[i]) ** 2

            return dist

        def get_closest_centroid(x, centroids):
            # Initialise the list to keep distances from each centroid
            centroid_distances = []

            # Loop over each centroid and compute the distance from data point.
            for centroid in centroids:
                dist = compute_l2_distance(x, centroid)
                centroid_distances.append(dist)

            # Get the index of the centroid with the smallest distance to the data point
            closest_centroid_index = min(
                range(len(centroid_distances)),
                key=lambda x: centroid_distances[x],
            )

            return closest_centroid_index

        def compute_sse(data, centroids, assigned_centroids):
            # Initialise SSE
            sse = 0

            # Compute the squared distance for each data point and add.
            for i, x in enumerate(data):
                # Get the associated centroid for data point
                centroid = centroids[assigned_centroids[i]]

                # Compute the Distance to the centroid
                dist = compute_l2_distance(x, centroid)

                # Add to the total distance
                sse += dist

            sse /= len(data)
            return sse

        # Number of dimensions in centroid
        num_centroid_dims = data.shape[1]

        # List to store SSE for each iteration
        sse_list = []

        # Loop over iterations
        for n in range(10):

            # Loop over each data point
            for i in range(len(data)):
                x = data[i]

                # Get the closest centroid
                closest_centroid = get_closest_centroid(x, centroids)

                # Assign the centroid to the data point.
                assigned_centroids[i] = closest_centroid

            # Loop over centroids and compute the new ones.
            for c in range(len(centroids)):
                # Get all the data points belonging to a particular cluster
                cluster_data = [
                    data[i] for i in range(len(data)) if assigned_centroids[i] == c
                ]

                # Initialise the list to hold the new centroid
                new_centroid = [0] * len(centroids[0])

                # Compute the average of cluster members to compute new centroid
                # Loop over dimensions of data
                for dim in range(num_centroid_dims):
                    dim_sum = [x[dim] for x in cluster_data]
                    dim_sum = sum(dim_sum) / len(dim_sum)
                    new_centroid[dim] = dim_sum

                # assign the new centroid
                centroids[c] = new_centroid

            # Compute the SSE for the iteration
            sse = compute_sse(data, centroids, assigned_centroids)
            sse_list.append(sse)

        cluster_members = []

        for c in range(len(centroids)):
            cluster_member = [
                data[i] for i in range(len(data)) if assigned_centroids[i] == c
            ]
            cluster_members.append(np.array(cluster_member))

        means = []

        for c in range(len(cluster_members)):
            means.append(np.mean(cluster_members[c], axis=0))

        print(means)
        return np.asarray(means), cluster_members

    def crop_image(self, pts, transform, img):
        rospy.logerr("The points for cropping before are", pts)
        pts = [self.model.project3dToPixel(transform.transform_point(a)) for a in pts]
        pts = np.array([[int(a[0]), int(a[1])] for a in pts], dtype=np.int32)
        pts = np.int32([pts])
        rospy.logerr("The points for cropping are", pts)
        mask = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        cv2.fillPoly(mask, pts, (255))
        res = cv2.bitwise_and(img, img, mask=mask)
        rect = cv2.boundingRect(pts)
        cropped = res[rect[1] : rect[1] + rect[3], rect[0] : rect[0] + rect[2]]
        return cropped

    def get_cluster_corners(self, cluster):
        avg_x = np.mean(cluster[:, 0])
        min_y = np.amin(cluster[:, 1])
        max_y = np.amax(cluster[:, 1])
        min_z = np.amin(cluster[:, 2])
        max_z = np.amax(cluster[:, 2])
        return np.asarray(
            [
                [avg_x, min_y, min_z],
                [avg_x, min_y, max_z],
                [avg_x, max_y, max_z],
                [avg_x, max_y, min_z],
            ],
        )

    async def crop_images(self, clusters, centers):
        image = await self.image_sub.get_next_message()
        image = self.bridge.imgmsg_to_cv2(image)            
        # cv2.imshow("Initial image", image) Not needed
        boat_to_cam = await self.tf_listener.get_transform(
            self.cam_frame,
            "wamv/base_link",
        )
        rospy.logerr('Cluster 0: ', clusters[0])
        rospy.logerr('Cluster 1: ', clusters[1])
        rospy.logerr('Cluster 2: ', clusters[2])
        
        left = self.crop_image(
            self.get_cluster_corners(clusters[0]),
            boat_to_cam,
            image,
        )
        middle = self.crop_image(
            self.get_cluster_corners(clusters[1]),
            boat_to_cam,
            image,
        )
        right = self.crop_image(
            self.get_cluster_corners(clusters[2]),
            boat_to_cam,
            image,
        )
        list = [left, middle, right]

        rospy.logerr(f"Left image shape: {left.shape}")
        rospy.logerr(f"Middle image shape: {middle.shape}")
        rospy.logerr(f"Right image shape: {right.shape}")

        h_min = min(a.shape[0] for a in list)
        counter = 0
        # Skip resize, its broken
        # resized = []
        # for im in list:
        #     if im.size > 0:  # Occasionally size is 0, causes errors
        #         resized = [
        #             cv2.resize(
        #                 im,
        #                 (int(im.shape[1] * h_min / im.shape[0]), h_min),
        #                 interpolation=cv2.INTER_CUBIC,
        #             )            
        #         ]
        #     else:
        #         print(f"Error: Image {counter} has size 0")
        #     counter += 1
        # concat = cv2.hconcat(resized)
        # msg = self.bridge.cv2_to_imgmsg(concat, encoding="rgb8")
        # self.contour_pub.publish(msg)

        return list
    
    async def shoot_projectile(self, img):
        rospy.logerr("- SHOOT PROJ REACHED -")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to the image
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Use Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours in the edged image
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Count to keep track of number of squares
        square_count = 0

        # Loop over the contours
        for contour in contours:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # If the approximated contour has 4 vertices, it's a square (or rectangle)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)

                # Calculate the aspect ratio
                aspect_ratio = float(w) / h

                # Check if the aspect ratio is close to 1 (square)
                if 0.95 <= aspect_ratio <= 1.05:
                    cv2.drawContours(img, [approx], -1, (0, 255, 0), 2)
                    square_count += 1
        
        # If three squares are not found (color and two holes) then bad
        if square_count != 3:
            rospy.logerr("Error: Incorrect number of squares detected")
            return

        # Display the result
        cv2.imshow("Squares Detected", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # ADD SHOOTING MECHANICS HERE

    def find_color(self, images, goal_color):
        # NOTE: An OpenCV window will open, close it to progress
        # Current iteration of find color works by looking through images,
        # then cropping images to the gray backboard (crop_images() does not always crop),
        # then it looks at a vertical line at the center of the image and averages the
        # non-gray values, returning Red Green Blue or Other.
        # Function does not always work, because the source image is weirdly cropped
        # or missing sometimes.

        count = 0

        # set a tolerance value for if we find two colors in the same image
        color_tolerance = 0.1

        for img in images:
            # Check if the image is empty before processing
            if img is None or img.size == 0:
                rospy.logerr("Error: Find color received image with 0 size, skipping")
                continue

            # Convert from BGR to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            

        # This code was used to detect the gray dock board and mask everything else out 
        # But it is not very useful, because there is a gray building in the background
            # # Split RGB channels
            # R, G, B = cv2.split(img_rgb)
            # # Thresholds for detecting gray
            # diff_threshold = 20  # Maximum difference allowed between R, G, and B
            # gray_lower = 80  # Lower bound for gray intensity
            # gray_upper = 200  # Upper bound for gray intensity

            # # Create a mask to find the gray board
            # mask = (
            #     (abs(R - G) < diff_threshold)
            #     & (abs(R - B) < diff_threshold)
            #     & (abs(G - B) < diff_threshold)
            #     & (gray_lower <= R)
            #     & (gray_upper >= R)  # Apply threshold for gray intensity
            #     & (gray_lower <= G)
            #     & (gray_upper >= G)
            #     & (gray_lower <= B)
            #     & (gray_upper >= B)
            # ).astype(
            #     "uint8",
            # ) * 255  # Convert the mask to a binary format (0 or 255)

            # dock_color = (0, 0, 0)  # Default value for color of dock

            # # Find contours of gray regions (the gray board)
            # contours, _ = cv2.findContours(
            #     mask,
            #     cv2.RETR_EXTERNAL,
            #     cv2.CHAIN_APPROX_SIMPLE,
            # )
            # if contours:
            #     # Largest contour is gray board
            #     largest_contour = max(contours, key=cv2.contourArea)
            #     x, y, w, h = cv2.boundingRect(largest_contour)

            #     # Uncomment to draw a rectangle around the detected gray board, show in openCV
            #     # cv2.rectangle(img_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #     # cv2.imshow('Gray Board Detection', img_rgb)

            #     # Crops Image around Gray Board
            #     cropped_img = img_rgb[y : y + h, x : x + w]
            #     cv2.imshow(f"{count}", cropped_img)

            # Now we find the color of the dock using the vertical centerline
            # Iterate over the vertical centerline from top to bottom
            cropped_img = img
            center_x = cropped_img.shape[1] // 2
            total_red = 0
            total_green = 0
            total_blue = 0
            num_pixels = 0  # Number of nongray pixels at center

            # Also get the number of pixels for each color
            green_pixels = 0
            red_pixels = 0
            blue_pixels = 0

            for center_y in range(cropped_img.shape[0]):
                current_color = cropped_img[
                    center_y,
                    center_x,
                ]  # Get RGB value at center
                current_color = current_color.astype(
                    np.int32,
                )  # Turns int8 colors into int32 to prevent overflow
                # Check if the color is weighted towards R G or B
                if (
                    current_color[2] > current_color[0] + current_color[1]
                    or current_color[1] > current_color[0] + current_color[2]
                    or current_color[0] > current_color[1] + current_color[2]
                ):
                    # If the color is not gray, add it to the total color
                    total_red += current_color[0]
                    total_green += current_color[1]
                    total_blue += current_color[2]
                    num_pixels += 1

                    # Count the number of pixels that are weighted towards R G or B
                    if current_color[0] > current_color[1] + current_color[2]:
                        red_pixels += 1
                    elif current_color[1] > current_color[0] + current_color[2]:
                        green_pixels += 1
                    elif current_color[2] > current_color[0] + current_color[1]:
                        blue_pixels += 1

            # Color at center is average of non gray pixels
            if num_pixels == 0: # Check if any colors found
                dock_color = (0,0,0)
            else:
                dock_color = (
                    total_red / num_pixels,
                    total_green / num_pixels,
                    total_blue / num_pixels,
                )

            # Get the ratios of each color
            red_ratio = red_pixels / num_pixels if num_pixels > 0 else 0
            green_ratio = green_pixels / num_pixels if num_pixels > 0 else 0
            blue_ratio = blue_pixels / num_pixels if num_pixels > 0 else 0

            # Now check if we find two colors outside of the tolerance by checking if there is an outright majority of one color
            if(count != 0): # PLEASE REMOVE THIS OUTER IF DURING ACTUAL TESTING, THIS IS TO SKIP THE YELLOW DOCK
                if (not (
                    (red_ratio - green_ratio > color_tolerance
                    and red_ratio - blue_ratio > color_tolerance)
                    or
                    (green_ratio - blue_ratio > color_tolerance
                    and green_ratio - red_ratio > color_tolerance)
                    or
                    (blue_ratio - green_ratio > color_tolerance
                    and blue_ratio - red_ratio > color_tolerance)
                    )
                ):
                    rospy.logerr(
                        f"Error: Found two colors in image {count} with ratios: R: {red_ratio}, G: {green_ratio}, B: {blue_ratio}")
                    # We return -1 signaling that this failed
                    return -1
            
            # Max ratio allowed between main color and other 2 values
            color_ratio = 0.9
            # Log the color (even after converting to RGB they still need to be BGR for this somehow)
            if (
                dock_color[0] > color_ratio * (dock_color[1] + dock_color[2])
                and goal_color == "Blue"
            ):
                # cv2.imshow('Blue Dock Detected', cropped_img)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                rospy.logerr("Detected color: Blue")
                return count
            elif (
                dock_color[1] > color_ratio * (dock_color[0] + dock_color[2])
                and goal_color == "Green"
            ):
                rospy.logerr("Detected color: Green")
                return count
            elif (
                dock_color[2] > color_ratio * (dock_color[0] + dock_color[1])
                and goal_color == "Red"
            ):
                # cv2.imshow('Red Dock Detected', cropped_img)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                rospy.logerr("Detected color: Red")
                return count
            else:
                rospy.logerr(f"Detected color: RGB{dock_color}")

            # Publish the image
            msg = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
            self.contour_pub.publish(msg)

            # Press key to destroy OpenCV window
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            count += 1

    def get_ogrid_coords(self, arr):
        return self.intup(self.ogrid_cpm * (np.asarray(arr) - self.ogrid_origin))[:2]

    def bbox_to_point(self, scale, rot, pos, xmul, ymul):
        return self.intup(
            self.ogrid_cpm
            * (
                self.get_point(
                    [(xmul * scale[0]) / 2, (ymul * scale[1]) / 2, scale[2]],
                    rot,
                    pos,
                )
                - self.ogrid_origin
            )[:2],
        )

    # returns True if side a is closest, False is side b is closest
    # This function works by drawing lines on the OGrid image. The OGrid is a 2D top down
    # image that takes the LIDAR points and colors a pixel in this grid is a point occupies it.
    # Using the bounding box of the LIDAR object, we create a bounding box around the 2D image of the dock.
    # Then, the center is calculated and lines are drawn to the middle point of each side of the bounding box.
    # The number of occupies squares are counted on the line from the object center to line middle. The line
    # which has the least amount of occupied space points to the open side of the dock.
    def calculate_correct_side(
        self,
        side_a: [float],
        side_b: [float],
        side_c: [float],
        side_d: [float],
        position: [float],
        rotation: [[float]],
        scale: [float],
    ) -> [float]:
        print("Finding ogrid center of mass")

        point1 = self.bbox_to_point(scale, rotation, position, -1, 1)
        point2 = self.bbox_to_point(scale, rotation, position, 1, 1)
        point3 = self.bbox_to_point(scale, rotation, position, 1, -1)
        point4 = self.bbox_to_point(scale, rotation, position, -1, -1)
        contours = np.array([point1, point2, point3, point4])

        center = self.calculate_center_of_mass(contours)
        mask = np.zeros(self.last_image.shape, np.uint8)

        bounding_rect = cv2.boundingRect(contours)
        x, y, w, h = bounding_rect

        image_or = np.zeros(self.last_image.shape, np.uint8)

        cv2.line(mask, self.get_ogrid_coords(side_a), center, (255, 255, 255), 2)
        side_a_and = cv2.bitwise_and(copy.deepcopy(self.last_image), mask)
        image_or = cv2.bitwise_or(image_or, side_a_and)
        mask = np.zeros(self.last_image.shape, np.uint8)

        cv2.line(mask, self.get_ogrid_coords(side_b), center, (255, 255, 255), 2)
        side_b_and = cv2.bitwise_and(copy.deepcopy(self.last_image), mask)
        image_or = cv2.bitwise_or(image_or, side_b_and)
        mask = np.zeros(self.last_image.shape, np.uint8)

        cv2.line(mask, self.get_ogrid_coords(side_c), center, (255, 255, 255), 2)
        side_c_and = cv2.bitwise_and(copy.deepcopy(self.last_image), mask)
        image_or = cv2.bitwise_or(image_or, side_c_and)
        mask = np.zeros(self.last_image.shape, np.uint8)

        cv2.line(mask, self.get_ogrid_coords(side_d), center, (255, 255, 255), 2)
        side_d_and = cv2.bitwise_and(copy.deepcopy(self.last_image), mask)
        image_or = cv2.bitwise_or(image_or, side_d_and)

        msg = self.bridge.cv2_to_imgmsg(image_or, encoding="rgb8")
        self.contour_pub.publish(msg)

        side_a_count = np.count_nonzero(side_a_and == np.asarray([255, 255, 255]))
        print(side_a_count)
        side_b_count = np.count_nonzero(side_b_and == np.asarray([255, 255, 255]))
        print(side_b_count)
        side_c_count = np.count_nonzero(side_c_and == np.asarray([255, 255, 255]))
        print(side_c_count)
        side_d_count = np.count_nonzero(side_d_and == np.asarray([255, 255, 255]))
        print(side_d_count)

        lowest = min([side_a_count, side_b_count, side_c_count, side_d_count])
        print("Center of mass found")
        if lowest == side_a_count:
            return side_a
        elif lowest == side_b_count:
            return side_b
        elif lowest == side_c_count:
            return side_c
        else:
            return side_d

    def ogrid_to_position(self, ogrid):
        ogrid = np.array(ogrid, dtype=np.float64)
        ogrid = ogrid / self.ogrid_cpm
        ogrid = ogrid + self.ogrid_origin[:2]
        return tuple(ogrid)

    def calculate_center_of_mass(self, points):
        bounding_rect = cv2.boundingRect(points)
        x, y, w, h = bounding_rect
        mask = np.zeros(self.last_image.shape, np.uint8)
        cv2.drawContours(mask, [points], -1, (255, 255, 255), -1, cv2.LINE_AA)
        masked = cv2.bitwise_and(self.last_image, mask)
        count = 0
        x_sum = 0
        y_sum = 0
        for i in range(x, x + w):
            for j in range(y, y + h):
                if (masked[j, i] == [255, 255, 255]).all():
                    x_sum = x_sum + i
                    y_sum = y_sum + j
                    count = count + 1
        x_sum = int(x_sum / count)
        y_sum = int(y_sum / count)
        return (x_sum, y_sum)

    def get_point(self, corner, rotation_matrix, center):
        return np.matmul(rotation_matrix, np.asarray(corner)) + np.asarray(center)

    async def find_dock_poi(self, hint: Optional[Pose] = None):
        print("Finding poi")

    # This function is used to find the position of the dock after the boat is near a POI
    async def find_dock(self):
        print("Searching for dock")
        msg = None
        while msg is None:
            try:
                # gets the largest object in the clusters database
                msg = await self.get_largest_object()
            except Exception:
                await self.move.forward(10).go()
        # label the largest LIDAR object in the PCODAR database "dock"
        await self.pcodar_label(msg.id, "dock")
        # if no pcodar objects, throw error, exit mission
        pose = pose_to_numpy(msg.pose)

        # return position of dock
        return pose

    def ogrid_cb(self, msg):
        self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.ogrid_origin = rosmsg_to_numpy(msg.info.origin.position)
        self.ogrid_cpm = 1 / msg.info.resolution

        image = 255 * np.greater(self.ogrid, 90).astype(np.uint8)
        grayImage = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        self.last_image = grayImage
        return
