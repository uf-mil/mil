#!/usr/bin/env python3
from typing import Optional, Sequence

import cv2
import mil_tools
import numpy as np
import rospy
import tf.transformations as trns
from mil_misc_tools.text_effects import fprint as _fprint
from nav_msgs.msg import OccupancyGrid, Odometry
from navigator_msgs.msg import PerceptionObject
from navigator_msgs.srv import (
    ObjectDBQuery,
    ObjectDBQueryRequest,
    ObjectDBQueryResponse,
)
from std_srvs.srv import Trigger, TriggerRequest

fprint = lambda *args, **kwargs: _fprint(time="", title="SIM", *args, **kwargs)


class DoOdom:
    """
    Republish odom  for lqrrt
    """

    def __init__(self, rand_size: float):
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=2)
        self.odom = None
        self.carrot_sub = rospy.Subscriber("/trajectory/cmd", Odometry, self.set_odom)

        fprint("Shaking hands and taking names.")
        rospy.sleep(1)

        # We need to publish an initial odom message for lqrrt
        start_ori = trns.quaternion_from_euler(0, 0, np.random.normal() * 3.14)
        start_pos = np.append(np.random.uniform(rand_size, size=(2)), 1)
        start_pose = mil_tools.numpy_quat_pair_to_pose(start_pos, start_ori)
        start_odom = Odometry()
        start_odom.header = mil_tools.make_header(frame="enu")
        start_odom.child_frame_id = "base_link"
        start_odom.pose.pose = start_pose
        self.odom_pub.publish(start_odom)

    def set_odom(self, msg):
        self.odom = mil_tools.pose_to_numpy(msg.pose.pose)
        self.odom_pub.publish(msg)


class Sim:
    """
    Attributes:
        ogrid_pub (Publisher): imports a grid-like object
        odom (DoOdom) : converts the "bf_size" size
        bf_size (float): stores a float version of the size
        min_t_spacing (float): stores a float version of spacing
        num_of_buoys (int): stores the number of buoys
        grid (object): this is a grid object
        resolution (float): sets the resolution of the grid
        height (float): sets a size that is three times the size of "bf_size"
        width (float): sets a size that is three times the size of "bf_size"
        origin (int): computed a numerical operation involving sizes
        publish_ogrid (Publisher): publishes a grid
        buoy_size (int): radius of buoy
        totem_size (int): radius of totem
        
    """
    def __init__(
        self, bf_size: float = 60, min_t_spacing: float = 9, num_of_buoys: int = 20
    ):
        self.ogrid_pub = rospy.Publisher("/ogrid", OccupancyGrid, queue_size=2)
        self.odom = DoOdom(bf_size)

        self.bf_size = bf_size
        self.min_t_spacing = min_t_spacing
        self.num_of_buoys = num_of_buoys

        # Some ogrid defs
        self.grid = None
        self.resolution = 0.3
        self.height = bf_size * 3
        self.width = bf_size * 3
        self.origin = mil_tools.numpy_quat_pair_to_pose(
            [-bf_size, -bf_size, 0], [0, 0, 0, 1]
        )

        self.publish_ogrid = lambda *args: self.ogrid_pub.publish(self.get_message())

        self.buoy_size = 1  # radius of buoy (m)
        self.totem_size = 1  # radius of totem (m)
        self.reseed(None)

        self.draw_buoys()
        self.draw_totems()
        self.publish_ogrid()

        # Now set up the database request service
        rospy.Service("/database/requests", ObjectDBQuery, self.got_request)
        rospy.Service("/reseed", Trigger, self.reseed)

        rospy.Timer(rospy.Duration(1), self.publish_ogrid)

    def _make_ogrid_transform(self):
        """
        Transforms points from ENU to ogrid frame coordinates

        Returns:
            lambda point: performs a dot operation for ogrid frame coordinates
        """
        self.grid = np.zeros(
            (self.height / self.resolution, self.width / self.resolution)
        )
        # Transforms points from ENU to ogrid frame coordinates
        self.t = np.array(
            [
                [1 / self.resolution, 0, -self.origin.position.x / self.resolution],
                [0, 1 / self.resolution, -self.origin.position.y / self.resolution],
                [0, 0, 1],
            ]
        )

        return lambda point: self.t.dot(np.append(point[:2], 1))[:2]

    def reseed(self, req: Optional[TriggerRequest]) -> None:
        """
        Generates buoys and totems and also checks for proximity

        Args:
            req (Optional[TriggerRequest]): the reseed operation is being performed on the request.
        """
        # Generate some buoys and totems
        buoy_positions = np.random.uniform(self.bf_size, size=(self.num_of_buoys, 2))
        self.ids = np.array([1, 45, 32, 55])
        self.totem_positions = np.array([[47, 41], [45, 15], [25, 40], [5, 12]])
        self.colors = [[0, 1, 0], [0, 0, 1], [0, 0, 1], [0, 0, 0]]

        # Let's make sure no buoys arae too close to the totems
        _buoy_positions = []
        for b in buoy_positions:
            if np.all(
                np.linalg.norm(self.totem_positions - b, axis=1) > self.min_t_spacing
            ):
                _buoy_positions.append(b)
        self.buoy_positions = np.array(_buoy_positions)
        print(len(self.buoy_positions))
        fprint(
            "Removed {} buoys that were too close to totems".format(
                len(self.buoy_positions) - len(_buoy_positions)
            )
        )
        # assert len(self.buoy_positions) > .5 * self.num_of_buoys, "Not enough buoys remain, try rerunning."
        if len(self.buoy_positions) < 0.5 * self.num_of_buoys:
            self.reseed(req)

        self.transform = self._make_ogrid_transform()

        self.draw_buoys()
        self.draw_totems()
        self.publish_ogrid()

    def position_to_object(
        self, position: np.ndarray, color: Sequence[int], id, name: str = "totem"
    ) -> PerceptionObject:
        """_summary_

        Args:
            position (np.ndarray): position of the object
            color (Sequence[int]): color of the sequence is passed in
            id (int): id of the object is passed in 
            name (str, optional): name of the object is passed in. Defaults to "totem".

        Returns:
            PerceptionObject: "PerceptionObject" instance is created with all the parameters being assigned. 
        """
        obj = PerceptionObject()
        obj.id = int(id)
        obj.header = mil_tools.make_header(frame="enu")
        obj.name = name
        obj.position = mil_tools.numpy_to_point(position)
        obj.color.r = color[0]
        obj.color.g = color[1]
        obj.color.b = color[2]
        obj.color.a = 1

        return obj

    def got_request(self, req: ObjectDBQueryRequest) -> ObjectDBQueryResponse:
        """Gets a request based on the name of the object

        Args:
            req (ObjectDBQueryRequest): object that is passed in for comparison purposes

        Returns:
            ObjectDBQueryResponse: response is submitted based on the request type
        """
        fprint(f"Request received {req.name}")
        if req.name in self.ids:
            index = np.argwhere(self.ids == req.name)
            objects = [
                self.position_to_object(
                    self.totem_positions[index], self.colors[index], req.name
                )
            ]
        elif req.name == "totem":
            objects = [
                self.position_to_object(p, c, i)
                for p, c, i in zip(self.totem_positions, self.colors, self.ids)
            ]
        elif req.name == "BuoyField":
            objects = [
                self.position_to_object(
                    [self.bf_size / 2, self.bf_size / 2, 0], [0, 0, 0], 0, "BuoyField"
                )
            ]
        else:
            return ObjectDBQueryResponse(found=False)

        return ObjectDBQueryResponse(objects=objects, found=True)

    def get_message(self) -> OccupancyGrid:
        """An ogrid component is being set with the assigned values. 

        Returns:
            OccupancyGrid: this type of grid is sent with all the passed-in data
        """
        if self.grid is None:
            fprint(
                "Ogrid was requested but no ogrid was found. Using blank.",
                msg_color="yellow",
            )
            self.grid = np.zeros(
                (self.height / self.resolution, self.width / self.resolution)
            )

        ogrid = OccupancyGrid()
        ogrid.header = mil_tools.make_header(frame="enu")
        ogrid.info.resolution = self.resolution
        ogrid.info.height, ogrid.info.width = self.grid.shape
        ogrid.info.origin = self.origin
        grid = np.copy(self.grid)
        ogrid.data = np.clip(grid.flatten() - 1, -100, 100).astype(np.int8)

        return ogrid

    def draw_buoys(self) -> None:
        """A buoys is being drawn out, especially with a circle. 
        """
        for b in self.buoy_positions:
            center = tuple(self.transform(b).astype(np.int32).tolist())
            cv2.circle(
                self.grid, center, int(self.buoy_size / self.resolution), 255, -1
            )

    def draw_totems(self) -> None:
        """A totem is being drawn with two separate circles. 
        """
        for b in self.totem_positions:
            center = tuple(self.transform(b).astype(np.int32).tolist())
            cv2.circle(
                self.grid, center, int(self.totem_size / self.resolution), 255, -1
            )
            cv2.circle(
                self.grid, center, int(0.5 * self.totem_size / self.resolution), -50, -1
            )


if __name__ == "__main__":
    fprint("Starting", msg_color="blue")
    rospy.init_node("Sim")

    Sim()
    rospy.spin()
