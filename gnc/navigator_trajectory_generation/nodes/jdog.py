#!/usr/bin/env python
from __future__ import division

import rospy
import actionlib
import tf

import numpy as np
import navigator_tools
from nav_msgs.msg import Odometry, OccupancyGrid
from navigator_msgs.msg import PoseTwistStamped
from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped
from navigator_msgs.msg import MoveToWaypointAction, MoveToWaypointActionFeedback, MoveToWaypointActionResult

import lqrrt
import setup_lqrrt as sl
from thread_queue import ThreadQueue


class NavPlanner(object):
    def __init__(self, plan_time):
        _time = lambda: rospy.Time.now().to_sec()

        self.traj_pub = rospy.Publisher("/trajectory", PoseTwistStamped, queue_size=1)
        self.traj_vis_pub = rospy.Publisher("/lqrrt/trajectory/visualize", PoseArray, queue_size=1)
        self.tree_vis_pub = rospy.Publisher("/lqrrt/tree", PoseArray, queue_size=1)
        self.impact_vis_pub = rospy.Publisher("/lqrrt/impact", PointStamped, queue_size=1)


        self.thread_queue = ThreadQueue(time=_time, max_size=20)

        self.goal = None
        self.state = None
        self.ogrid = None
        self.done = True

        rospy.Subscriber("/odom", Odometry, lambda msg: setattr(self, "state", self.state_from_odom(msg)))
        rospy.Subscriber("/ogrid_batcave_throttle", OccupancyGrid, self.ogrid_cb)

        # Self crew
        self.goal_buffer = [2, 2, np.inf, np.inf, np.inf, np.inf]
        self.error_tol = np.copy(self.goal_buffer) / 8
        self.velmax_pos_plan = np.array([1, 0.4, 0.2])  # (m/s, m/s, rad/s), body-frame forward
        self.velmax_neg_plan = np.array([-0.65, -0.4, -0.2])  # (m/s, m/s, rad/s), body-frame backward
        self.goal_bias = [0, 0, 0, 0, 0, 0]
        self.original_magic_rudder = sl.magic_rudder

        self.plan_time = plan_time

        self.constraints = lqrrt.Constraints(nstates=sl.nstates, ncontrols=sl.ncontrols,
                                             goal_buffer=self.goal_buffer, is_feasible=self.is_feasible)

        self.planner = lqrrt.Planner(sl.dynamics, sl.lqr, self.constraints,
                                     horizon=2, dt=0.1, FPR=0.9,
                                     error_tol=self.error_tol, erf=sl.erf,
                                     min_time=plan_time, max_time=plan_time,
                                     max_nodes=1E5, system_time=_time)

        self.start_planner()
        self.keep_alive()

    def keep_alive(self):
        # Main loop used to clear the queue
        while not rospy.is_shutdown():
            self.thread_queue.next_if_possible()
            rospy.sleep(.05)

    def start_planner(self):
        '''
        Waits for odom to come in then will start planning with that data.
        Assumes station holding at start
        '''
        while self.state is None and not rospy.is_shutdown():
            print "Planner waiting for odom"
            rospy.sleep(.1)

        print "Found Odom!"

        # Set things
        self.goal = self.state

        # Set up first plan
        sample_space = self.sample_space(self.state)
        self.planner.set_goal(self.goal)
        self.planner.update_plan(self.state, sample_space, goal_bias=self.goal_bias)
        self.last_update_time = rospy.Time.now()

        # Set up timers
        rospy.Timer(rospy.Duration(.05), self.publish_trajectory)  # 20hz
        rospy.Timer(rospy.Duration(2), self.diag_pub)

    def state_from_odom(self, msg):
        # x, y, heading, vx, vy, w
        pose, twist = navigator_tools.odometry_to_numpy(msg)[:2]
        position, orientation = pose
        linear, angular = twist

        x, y = position[:2]
        heading = tf.transformations.euler_from_quaternion(orientation)[2]
        vx, vy = linear[:2]
        w = angular[2]

        return [x, y, heading, vx, vy, w]

    def update_plan(self, *args):
        if self.done: return
        print "UPDATING PLAN"

        # Plan for where we will be when we're done planning
        T = (rospy.Time.now() - self.last_update_time).to_sec()
        future_state = self.planner.get_state(T + self.plan_time)
        print "Planned start state", future_state

        # Generate a sample space for that future state and then plan from there
        sample_space = self.sample_space(future_state)
        self.planner.update_plan(future_state, sample_space, goal_bias=self.goal_bias)
        self.last_update_time = rospy.Time.now()

        # We want to re-plan when we are `self.plan_time` seconds away from the end of the current plan
        next_time_to_run = rospy.Time.now().to_sec() + self.planner.T - self.plan_time
        self.thread_queue.put(self.update_plan, next_time_to_run)
        print "Updating again in {}s".format(next_time_to_run - rospy.Time.now().to_sec())
        print "DONE UPDATING"

    def diag_pub(self, *args):
        # Run on a timer
        #print "pubbing debug"
        self.publish_trajectory_array()
        self.publish_tree_array()

    def publish_trajectory(self, *args):
        # Run on a timer
        T = (rospy.Time.now() - self.last_update_time).to_sec()

        traj = self.goal if self.done else self.planner.get_state(T)

        pose_twist = PoseTwistStamped()
        pose_twist.header = navigator_tools.make_header(frame='enu')
        q = tf.transformations.quaternion_from_euler(0, 0, traj[2])
        pose_twist.posetwist.pose = navigator_tools.numpy_quat_pair_to_pose(np.array([traj[0], traj[1], 0]), q)
        pose_twist.posetwist.twist = navigator_tools.numpy_to_twist([traj[3], traj[4], 0], [0, 0, traj[5]])

        self.traj_pub.publish(pose_twist)

    def publish_trajectory_array(self):
        pose_array = PoseArray()
        pose_array.header = navigator_tools.make_header(frame='enu')
        pose_array_list = []

        for x in self.planner.x_seq:
            traj = x[:3]  # x, y, theta
            pose = PoseTwistStamped()
            q = tf.transformations.quaternion_from_euler(0, 0, traj[2])
            pose_array_list.append(navigator_tools.numpy_quat_pair_to_pose(np.array([traj[0], traj[1], 0]), q))

        pose_array.poses = pose_array_list
        self.traj_vis_pub.publish(pose_array)

    def publish_tree_array(self):
        pose_array = PoseArray()
        pose_array.header = navigator_tools.make_header(frame='enu')
        pose_array_list = []

        for ID in xrange(self.planner.tree.size):
            traj = self.planner.tree.state[ID]
            pose = PoseTwistStamped()
            q = tf.transformations.quaternion_from_euler(0, 0, traj[2])
            pose_array_list.append(navigator_tools.numpy_quat_pair_to_pose(np.array([traj[0], traj[1], 0]), q))

        pose_array.poses = pose_array_list
        self.tree_vis_pub.publish(pose_array)

    def path_still_feasible(self):
        for i, (x, u) in enumerate(zip(self.planner.x_seq, self.planner.u_seq)):
            if not self.is_feasible(x, u):
                # Need to re-plan ASAP!
                time_to_hit = i * self.planner.dt
                if time_to_hit <= self.plan_time + .5:
                    # STOP AND REPLAN
                    pass
                else:
                    ps = PointStamped()
                    ps.header = navigator_tools.make_header(frame='enu')
                    ps.point = navigator_tools.numpy_to_point(x[:3])
                    self.impact_vis_pub.publish(ps)
                    print ps

                    # Queue up a clear then a re-plan
                    self.thread_queue.put(self.update_plan, rospy.Time.now().to_sec() - 1)
                    self.thread_queue.put(self.thread_queue.clear, rospy.Time.now().to_sec())

                return

    def is_feasible(self, x, u):
        # Reject going too fast
        # for i, v in enumerate(x[3:]):
        #     if v > self.velmax_pos_plan[i] or v < self.velmax_neg_plan[i]:
        #         return False

        if self.ogrid is None:
            # If there's no ogrid yet, anywhere is valid
            return True

        # Body to world
        c, s = np.cos(x[2]), np.sin(x[2])
        R = np.array([[c, -s],
                      [s,  c]])

        # Boat verts in world frame
        points = x[:2] + R.dot(sl.vps).T

        # Check for collision
        indicies = (self.ogrid_cpm * (points - self.ogrid_origin)).astype(np.int64)
        try:
            grid_values = self.ogrid[indicies[:,1], indicies[:,0]]
        except IndexError:
            print "WOAH NELLY! Search exceeded ogrid size."
            return False

        # Assuming anything greater than 90 is a hit
        return np.all(grid_values < 90)

    def ogrid_cb(self, msg):
        """
        Updated the stored ogrid array and origin vector.
        """
        self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.ogrid_cpm = 1 / msg.info.resolution

        self.thread_queue.put(self.path_still_feasible, rospy.Time.now().to_sec())


    def sample_space(self, start):
        buff = 40
        xs = (min([start[0], self.goal[0]]) - buff, max([start[0], self.goal[0]]) + buff)
        ys = (min([start[1], self.goal[1]]) - buff, max([start[1], self.goal[1]]) + buff)
        return [xs,
                ys,
                (0, 0),
                (-abs(self.velmax_neg_plan[0]), self.velmax_pos_plan[0]),
                (-abs(self.velmax_neg_plan[1]), self.velmax_pos_plan[1]),
                (-abs(self.velmax_neg_plan[2]), self.velmax_pos_plan[2])]


class ActionLink(NavPlanner):
    _feedback = MoveToWaypointActionFeedback().feedback
    _result = MoveToWaypointActionResult().result

    def __init__(self, plan_time):
        self._time = lambda: rospy.Time.now().to_sec()
        self.wp_server = actionlib.SimpleActionServer("/move_to",
                MoveToWaypointAction, execute_cb=self.got_wp, auto_start=False)
        self.wp_server.start()

        # Contains most of the class variables used here and the interface to lqrrt
        super(ActionLink, self).__init__(plan_time)

    def got_wp(self, a_goal):
        print "Goal received"
        print "Station Hold:", a_goal.station_hold

        des_state = self.goal_from_ac_goal(a_goal)
        goal = self.goal if a_goal.station_hold else des_state
        bias = a_goal.goal_bias if a_goal.goal_bias else self.goal_bias
        goal_bias = [bias, bias, 0, 0, 0, 0]
        self.planner.set_goal(goal)

        sample_space = self.sample_space(self.state)
        plan = lambda: self.planner.update_plan(self.state, sample_space, goal_bias=goal_bias)
        self.thread_queue.clear()
        self.thread_queue.put(plan)

        print "planning..."
        rospy.sleep(self.plan_time + .5)
        print "done planning!"
        self.last_update_time = rospy.Time.now()
        next_time_to_run = self._time() + self.planner.T - self.plan_time
        self.thread_queue.put(self.update_plan, next_time_to_run)
        print "Updating in {}s".format(next_time_to_run - self._time())

        self.goal = goal
        self.goal_bias = goal_bias

        self.done = False
        print self.state
        print "goal:", self.goal
        print "bias:", self.goal_bias

        while not rospy.is_shutdown() and not self.planner._in_goal(self.state):
            self.publish_feedback()
            
            if self.wp_server.is_preempt_requested():
                print "PREEMPT REQUEST"
                self.safe_cancel()
                self.wp_server.set_preempted()
                return

            rospy.sleep(.1)

        self.thread_queue.clear()
        self.done = True

        self.wp_server.set_succeeded(self._result)

    def safe_cancel(self):
        self.goal = self.state
        self.done = True
        self.thread_queue.clear()

    def publish_feedback(self):
        time_into_plan = self._time() - self.last_update_time.to_sec()
        self._feedback.time_to_completion = self.planner.T - time_into_plan
        self._feedback.time_to_replan = self.planner.T - time_into_plan - self.plan_time
        self._feedback.time_for_replan = self.plan_time
        return self.wp_server.publish_feedback(self._feedback)

        # float32 time_to_completion
        # float32 time_to_replan
        # float32 time_for_replan
        # bool replanning
        # bool refeasing
        # bool traj_reached_goal


    def goal_from_ac_goal(self, goal):
        # x, y, heading, vx, vy, w
        position, orientation = navigator_tools.pose_to_numpy(goal.target.pose)
        linear, angular = navigator_tools.twist_to_numpy(goal.target.twist)

        x, y = position[:2]
        heading = tf.transformations.euler_from_quaternion(orientation)[2]
        vx, vy = linear[:2]
        w = angular[2]

        return [x, y, heading, vx, vy, w]



if __name__ == "__main__":
    rospy.init_node("navigation_planner")
    ac = ActionLink(rospy.get_param("plan_time", 2))
    #n = NavPlanner()

    rospy.spin()
