#! /usr/bin/python3

from copy import deepcopy
from typing import List, Optional, Tuple

import cv2
import genpy
import mil_tools
import numpy as np
import rclpy
import rclpy.duration
import rclpy.timer
import tf.transformations as trns
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from mil_misc_tools.text_effects import fprint as _fprint
from nav_msgs.msg import OccupancyGrid, Odometry
from navigator_msg_multiplexer.cfg import OgridConfig
from navigator_path_planner import params
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger

logger = rclpy.logging.get_logger("ogrid_arbiter")


# Wow what a concept
def fprint(*args, **kwargs):
    return _fprint(*args, title="OGRID_ARB", **kwargs)


def make_ogrid_transform(ogrid: OccupancyGrid) -> np.array:
    """
    Returns a matrix that transforms a point in ENU to this ogrid.
    Invert the result to get ogrid -> ENU.

    Args:
        ogrid: OccupancyGrid - The occupancy grid to make a transform of.

    Returns:
        A numpy array (np.array) representing an ogrid
        with a transformed ENU.
    """
    resolution = ogrid.info.resolution
    origin = mil_tools.pose_to_numpy(ogrid.info.origin)[0]

    # Transforms points from ENU to ogrid frame coordinates
    t = np.array(
        [
            [1 / resolution, 0, -origin[0] / resolution],
            [0, 1 / resolution, -origin[1] / resolution],
            [0, 0, 1],
        ],
    )
    return t


def numpyify(ogrid: OccupancyGrid) -> np.array:
    """
    Converts an occupancy grid to a pure numpy array.

    Args:
        ogrid: OccupancyGrid - The occupancy grid to convert.

    Returns:
        A numpy array (np.array) representing the original
        ogrid.
    """
    return np.array(ogrid.data).reshape(ogrid.info.height, ogrid.info.width)


def transform_enu_to_ogrid(enu_points: List[int], grid: OccupancyGrid) -> np.ndarray:
    """
    Converts an ENU point into the global ogrid's frame.

    Args:
        enu_points: List[List[int]] - A list of ENU points
        to add to the global ogrid frame.
        grid: OccupancyGrid - The global occupancy grid.

    Returns:
        np.ndarray - ???
    """
    enu_points_np = np.array(enu_points)

    if enu_points_np.size > 3:
        enu_points_np[:, 2] = 1

    t = make_ogrid_transform(grid)
    return t.dot(np.array(enu_points_np).T).T


def transform_ogrid_to_enu(grid_points: List[int], grid: OccupancyGrid) -> np.ndarray:
    """
    Converts an ogrid cell into the ENU frame.

    Args:
        grid_points: List[List[int]] - A list of ogrid points that
        will be converted to the ENU frame.
        grid: OccupancyGrid - The occupancy grid representing the ENU frame.

    Returns:
        np.ndarray - ???
    """
    grid_points_np = np.array(grid_points)

    if grid_points_np.size > 3:
        grid_points_np[:, 2] = 1

    t = np.linalg.inv(make_ogrid_transform(grid))
    return t.dot(np.array(grid_points_np).T).T


def transform_between_ogrids(
    grid1_points: List[int],
    grid1: OccupancyGrid,
    grid2: OccupancyGrid,
) -> np.ndarray:
    """
    ???

    Args:
        grid1_points: List[List[int]] - ???
        grid1: OccupancyGrid - ???
        grid2: OccupancyGrid - ???

    Returns:
        np.ndarray - ???
    """
    enu = transform_ogrid_to_enu(grid1_points, grid1)
    return transform_enu_to_ogrid(enu, grid2)


def get_enu_corners(grid) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns the bottom left and top right (?) points in the ENU frame.

    Args:
        grid: OccupancyGrid - The occupancy grid holding the
        ENU frame.

    Returns:
        A tuple holding the bottom left and the top right points
        in the ENU frame.
    """
    grid_to_enu = np.linalg.inv(make_ogrid_transform(grid))
    min = grid_to_enu.dot([0, 0, 1])
    max = grid_to_enu.dot([grid.info.height, grid.info.width, 1])
    return (min, max)


class OGrid:
    """
    Represents a class holding an occupancy grid. The class
    is likely used to store an occupancy grid in a dynamic context,
    such as when dynamic reconfigure is updated.
    """

    last_message_stamp: Optional[genpy.Time]
    topic: str
    nav_ogrid: Optional[OccupancyGrid]
    np_map: Optional[np.ndarray]
    replace: bool
    subscriber: rclpy.Subscriber

    def __init__(self, topic: str, replace: bool = False, frame_id: str = "enu"):
        # Assert that the topic is valid
        self.last_message_stamp = None
        self.topic = topic
        self.nav_ogrid = None  # Last received OccupancyGrid message
        self.np_map = None  # Numpy version of last received OccupancyGrid message
        self.replace = replace
        self.subscriber = rclpy.create_subscription(
            OccupancyGrid,
            topic,
            self.subscriber_callback,
            1,
        )

    @property
    def callback_delta(self) -> float:
        """
        The difference between the time of the last message
        and now, in seconds.

        Returns:
            A float representing the number of seconds.
        """
        if self.last_message_stamp is None:
            return 0
        return (rclpy.Time.now() - self.last_message_stamp).to_sec()

    def subscriber_callback(self, ogrid: OccupancyGrid):
        """
        The callback function for the topic subscriber. The callback
        will update the class with the last message stamp time and
        most recent occupany grid.
        """
        self.last_message_stamp = ogrid.header.stamp
        self.nav_ogrid = ogrid
        self.np_map = numpyify(ogrid)


class OGridServer:
    odom: Optional[Tuple[np.ndarray, np.ndarray]]

    def __init__(
        self,
        frame_id: str = "enu",
        map_size: int = 500,
        resolution: float = 0.3,
        rate: int = 1,
    ):
        self.frame_id = frame_id
        self.ogrids = {}
        self.odom = None

        # Some default values
        self.plow = True
        self.plow_factor = 0
        self.ogrid_min_value = -1
        self.draw_bounds = False
        self.resolution = resolution
        self.ogrid_timeout = 2
        self.enforce_bounds = False
        self.enu_bounds = [[0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]]

        # Default to centering the ogrid
        position = np.array(
            [-(map_size * resolution) / 2, -(map_size * resolution) / 2, 0],
        )
        quaternion = np.array([0, 0, 0, 1])

        self.origin = mil_tools.numpy_quat_pair_to_pose(position, quaternion)
        self.global_ogrid = self.create_grid((map_size, map_size))

        node.create_subscription(Odometry, "/odom", self.set_odom)
        self.publisher = node.create_publisher(OccupancyGrid, "/ogrid_master", 1)

        self.ogrid_server = Server(OgridConfig, self.dynamic_config_callback)
        self.dynam_client = Client("bounds_server", config_callback=self.bounds_cb)

        node.create_service(Trigger, "~center_ogrid", self.center_ogrid)
        node.create_timer(1.0 / rate, self.publish)

    def set_odom(self, msg: Odometry) -> np.ndarray:
        """
        Sets the odom variable to a numpy array, and returns it.

        Used by the subscriber to /odom as a callback in order to parse
        and store received messages.

        Args:
            msg: Odometry - The message from the /odom topic

        Returns:
            np.ndarray - The converted pose in a numpy array.
        """
        return setattr(self, "odom", mil_tools.pose_to_numpy(msg.pose.pose))

    def center_ogrid(self, _) -> dict:
        """
        Serves as the callback to the center_ogrid service,
        and returns whether the operation was successful. A
        complete operation sets the center of the ogrid to the
        current odometry position.

        If no odometry position is found, then success is returned
        as False.

        Returns:
            A dict containing values that will be fed into a
            complete trigger response.
        """
        if self.odom is None:
            return {"success": False, "message": "odom not received"}

        dim = -(self.map_size[0] * self.resolution) / 2
        new_org = self.odom[0] + np.array([dim, dim, 0])

        # Update the dynamic configuration
        config = {}
        config["origin_x"] = float(new_org[0])
        config["origin_y"] = float(new_org[1])
        config["set_origin"] = True
        self.ogrid_server.update_configuration(config)

        return {"success": True}

    def bounds_cb(self, config: dict) -> None:
        """
        Update bounds which may be drawn in ogrid when
        dynamic reconfigure updates.

        Args:
            config: dict - The update changes from dynamic
            reconfigure to be handled.
        """
        logger.info("BOUNDS UPDATED")

        self.enu_bounds = [
            [config["x1"], config["y1"], 1],
            [config["x2"], config["y2"], 1],
            [config["x3"], config["y3"], 1],
            [config["x4"], config["y4"], 1],
            [config["x1"], config["y1"], 1],
        ]
        self.enforce_bounds = config["enforce"]

    def dynamic_config_callback(self, config: dict, _) -> dict:
        """
        Accepts changes in the ogrid dynamic configuration.

        Args:
            config: dict - The new configuration setup.
            level (not used) - The result of the level operation completed
            on all parameters by Dynamic Reconfigure.

        Returns:
            The updated dict containing no changes to the configuration.
        """
        fprint("OGRID DYNAMIC CONFIG UPDATE!", msg_color="blue")

        # Receive and parse topic list
        topics = config["topics"].replace(" ", "").split(",")
        replace_topics = config["replace_topics"].replace(" ", "").split(",")

        # Update OGrids with new configuration changes
        new_grids = {}
        for topic in topics:
            new_grids[topic] = (
                OGrid(topic) if topic not in self.ogrids else self.ogrids[topic]
            )
        for topic in replace_topics:
            new_grids[topic] = (
                OGrid(topic, replace=True)
                if topic not in self.ogrids
                else self.ogrids[topic]
            )

        self.ogrids = new_grids

        map_size = map(int, (config["height"], config["width"]))
        self.map_size = map_size

        self.plow = config["plow"]
        self.plow_factor = config["plow_factor"]
        self.ogrid_min_value = config["ogrid_min_value"]
        self.draw_bounds = config["draw_bounds"]
        self.resolution = config["resolution"]
        self.ogrid_timeout = config["ogrid_timeout"]

        if config["set_origin"]:
            fprint("Setting origin!")
            position = np.array([config["origin_x"], config["origin_y"], 0])
            quaternion = np.array([0, 0, 0, 1])
            self.origin = mil_tools.numpy_quat_pair_to_pose(position, quaternion)

        else:
            position = np.array(
                [
                    -(map_size[1] * self.resolution) / 2,
                    -(map_size[0] * self.resolution) / 2,
                    0,
                ],
            )
            quaternion = np.array([0, 0, 0, 1])
            self.origin = mil_tools.numpy_quat_pair_to_pose(position, quaternion)

        self.global_ogrid = self.create_grid(map_size)

        # Return dynamic configuration with no changes
        return config

    def create_grid(self, map_size: Tuple[int, int]) -> OccupancyGrid:
        """
        Generates a blank ogrid for the low price of $9.95!

        Args:
            map_size: Tuple[int, int] - The size of the map
            to generate in the ogrid.
        """
        ogrid = OccupancyGrid()
        ogrid.header.stamp = rclpy.Time.now()
        ogrid.header.frame_id = self.frame_id

        ogrid.info.resolution = self.resolution
        ogrid.info.width = map_size[1]
        ogrid.info.height = map_size[0]

        ogrid.info.origin = self.origin

        # TODO: Make sure this produces the correct sized ogrid
        ogrid.data = np.full((map_size[1], map_size[0]), -1).flatten()

        return ogrid

    def publish(self, timer_event):
        """
        Publishes the final ogrid to the ROS topic. Called by
        a rclpy.Timer set to a specific rate.

        Args:
            timer_event: The TimerEvent sent by the rclpy Timer
            upon each call by the timer.
        """
        global_ogrid = deepcopy(self.global_ogrid)
        np_grid = numpyify(global_ogrid)

        # Global ogrid (only compute once)
        corners = get_enu_corners(global_ogrid)
        index_limits = transform_enu_to_ogrid(corners, global_ogrid)[:, :2]

        g_x_min = index_limits[0][0]
        g_x_max = index_limits[1][0]
        g_y_min = index_limits[0][1]
        g_y_max = index_limits[1][1]

        to_add = [o for o in self.ogrids.values() if not o.replace]
        to_replace = [o for o in self.ogrids.values() if o.replace]

        for ogrids in [to_add, to_replace]:
            for ogrid in ogrids:
                # Hard coded 5 second timeout - probably no need to reconfig this.
                if ogrid.nav_ogrid is None or ogrid.callback_delta > self.ogrid_timeout:
                    # fprint("Ogrid too old!")
                    continue

                # Proactively checking for errors.
                # This should be temporary but probably won't be.
                l_h, l_w = ogrid.nav_ogrid.info.height, ogrid.nav_ogrid.info.width
                g_h, g_w = global_ogrid.info.height, global_ogrid.info.width
                if l_h > g_h or l_w > g_w:
                    fprint(
                        "Proactively preventing errors in ogrid size.",
                        msg_color="red",
                    )
                    new_size = max(l_w, g_w, l_h, g_h)
                    self.global_ogrid = self.create_grid((new_size, new_size))

                # Local Ogrid (get everything in global frame though)
                corners = get_enu_corners(ogrid.nav_ogrid)
                index_limits = transform_enu_to_ogrid(corners, ogrid.nav_ogrid)
                index_limits = transform_between_ogrids(
                    index_limits,
                    ogrid.nav_ogrid,
                    global_ogrid,
                )[:, :2]

                l_x_min = index_limits[0][0]
                l_x_max = index_limits[1][0]
                l_y_min = index_limits[0][1]
                l_y_max = index_limits[1][1]

                xs = np.sort([g_x_max, g_x_min, l_x_max, l_x_min])
                ys = np.sort([g_y_max, g_y_min, l_y_max, l_y_min])
                # These are indices in cell units
                start_x, end_x = np.round(xs[1:3])  # Grabbing indices 1 and 2
                start_y, end_y = np.round(ys[1:3])

                # Should be indices
                l_ogrid_start = transform_between_ogrids(
                    [start_x, start_y, 1],
                    global_ogrid,
                    ogrid.nav_ogrid,
                )

                # fprint("ROI {},{} {},{}".format(start_x, start_y, end_x, end_y))
                index_width = (
                    l_ogrid_start[0] + end_x - start_x
                )  # I suspect rounding will be a source of error
                index_height = l_ogrid_start[1] + end_y - start_y
                # fprint("width: {}, height: {}".format(index_width, index_height))
                # fprint("Ogrid size: {}, {}".format(ogrid.nav_ogrid.info.height, ogrid.nav_ogrid.info.width))

                to_add = ogrid.np_map[
                    l_ogrid_start[1] : index_height,
                    l_ogrid_start[0] : index_width,
                ]

                # fprint("to_add shape: {}".format(to_add.shape))

                # Make sure the slicing doesn't produce an error
                end_x = start_x + to_add.shape[1]
                end_y = start_y + to_add.shape[0]

                try:
                    # fprint("np_grid shape: {}".format(np_grid[start_y:end_y, start_x:end_x].shape))
                    fprint(f"{ogrid.topic}, {ogrid.replace}")
                    if ogrid.replace:
                        np_grid[start_y:end_y, start_x:end_x] = to_add
                    else:
                        np_grid[start_y:end_y, start_x:end_x] += to_add

                except Exception as e:
                    fprint(
                        "Exception caught, probably a dimension mismatch:",
                        msg_color="red",
                    )
                    print(e)
                    fprint(
                        f"w: {global_ogrid.info.width}, h: {global_ogrid.info.height}",
                        msg_color="red",
                    )

        if self.draw_bounds and self.enforce_bounds:
            ogrid_bounds = transform_enu_to_ogrid(self.enu_bounds, global_ogrid).astype(
                np.int32,
            )
            for i, point in enumerate(ogrid_bounds[:, :2]):
                if i == 0:
                    last_point = point
                    continue
                cv2.line(np_grid, tuple(point), tuple(last_point), 100, 3)
                last_point = point

        if self.plow:
            self.plow_snow(np_grid, global_ogrid)

        # Clip and flatten grid
        np_grid = np.clip(np_grid, self.ogrid_min_value, 100)
        global_ogrid.data = np_grid.flatten().astype(np.int8)

        # Publish the ogrid to the topic
        self.publisher.publish(global_ogrid)

    def plow_snow(self, np_grid: np.ndarray, ogrid: OccupancyGrid) -> np.ndarray:
        """
        Remove region around the boat so we never touch an occupied cell
        (making lqrrt not break if something touches us). If the boat's location
        is not known, then the np_grid argument is returned.

        Args:
            np_grid: np.ndarray - ???
            ogrid: OccupancyGrid - ???

        Returns:
            np.ndarray - ???
        """
        if self.odom is None:
            return np_grid

        position, orientation = self.odom

        yaw_rot = trns.euler_from_quaternion(orientation)[2]  # rads
        boat_width = params.boat_length + params.boat_buffer + self.plow_factor  # m
        boat_height = params.boat_width + params.boat_buffer + self.plow_factor  # m

        x, y, _ = transform_enu_to_ogrid([position[0], position[1], 1], ogrid)
        theta = yaw_rot
        w = boat_width / ogrid.info.resolution
        h = boat_height / ogrid.info.resolution

        box = cv2.boxPoints(((x, y), (w, h), np.degrees(theta)))
        box = np.int0(box)
        cv2.drawContours(np_grid, [box], 0, 0, -1)

        # Draw a "boat" in the ogrid
        boat_width = params.boat_length + params.boat_buffer
        boat_height = params.boat_width + params.boat_buffer

        x, y, _ = transform_enu_to_ogrid([position[0], position[1], 1], ogrid)
        w = boat_width / ogrid.info.resolution
        h = boat_height / ogrid.info.resolution

        box = cv2.boxPoints(((x, y), (w, h), np.degrees(theta)))
        box = np.int0(box)
        cv2.drawContours(np_grid, [box], 0, 40, -1)

        return np_grid


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("ogrid_server")

    ### Parameters
    topics_descriptor = ParameterDescriptor(
        type="string",
        description="A comma delimited list of topics to subscribe to for ogrid updates.",
    )
    node.declare_parameter(
        "topics",
        "ogrid, mission_ogrid, draw_ogrid",
        topics_descriptor,
    )

    ### Finalizing server
    og_server = OGridServer()
    rclpy.spin(node)
