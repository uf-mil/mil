#!/usr/bin/env python
'''Monte-Carlo controller verification
'''
import rospy
import numpy as np
import time
import geometry_msgs.msg as geometry_msgs
import sub8_ros_tools as sub8_utils
from sub8_msgs.msg import Trajectory, Waypoint
import sub8_ros_tools as sub8_utils
from sub8_simulation.srv import SimSetPose
import nav_msgs.msg as nav_msgs
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # flake8: noqa
from scipy import cluster


class VerifyController(object):
    _sampling_period = rospy.Duration(0.1)
    def __init__(self, num_runs=4, time_limit=10, do_plot=False):
        '''Verify that a controller is...

            Stable:
                - after T_end seconds, the state remains with an n-ball of radius r
                - i.e. convergence = max(map(np.linalg.norm, points[::-T] - np.avg(points[::-T])))

            Converging:
                - iff convergence[t] < convergence[t - 1]

            Useful:
                - || centroid(points[::-T]) - centroid(points[::-T]) ||

        Arguments:
            num_runs: How many runs to do
            time_limit: How much time to allow each run to last
        '''
        self.last_sample = rospy.Time.now()

        self.do_plot = do_plot
        self.num_runs = num_runs
        self.time_limit = time_limit

        self.r_start = time.time()

        # TODO: State history
        self.start_time = rospy.Time.now()
        self.T = 100
        self.max_length = 1000

        self.all_histories = []
        self.cur_state_history = []
        self.odom_sub = rospy.Subscriber('/truth/odom', nav_msgs.Odometry, self.odom_cb, queue_size=1)
        self.target_sub = rospy.Subscriber('/trajectory', Trajectory, self.traj_cb)
        self.target_pub = rospy.Publisher('/trajectory', Trajectory, latch=True, queue_size=1)
        self.target_state = None

        rospy.wait_for_service('sim/vehicle/set_pose', timeout=5.0)
        self.set_pose = rospy.ServiceProxy('sim/vehicle/set_pose', SimSetPose)

    def set_random_pose(self, center, scale):
        '''Set a random position, and for now constant orientation
        TODO: Random orientation
        '''
        pos = center + (np.random.random(3) * scale)
        self.set_pose(
            pose=geometry_msgs.Pose(
                position=pos,
                orientation=(0.0, 0.0, 0.0, 1.0),
            )
        )

    def set_target(self, pos, orientation=(0.0, 0.0, 0.0, 1.0)):
        '''TODO:
            Add linear/angular velocity
            Publish a trajectory
        '''
        # self.target = make_pose_stamped(position=pos, orientation=orientation)
        target_msg = Trajectory(
            header=sub8_utils.make_header(frame='/body'),
            trajectory=[Waypoint(
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Vector3(*pos),
                    orientation=geometry_msgs.Quaternion(*orientation)
                )
            )]
        )
        self.target_pub.publish(target_msg)


    def traj_cb(self, msg):
        pose, twist = sub8_utils.posetwist_to_numpy(msg.trajectory[0])
        self.target_state = pose[0]

    def odom_cb(self, msg):
        time_of = rospy.Time.now()
        if (time_of - self.last_sample) < self._sampling_period:
            return
        else:
            self.last_sample = time_of

        pose, twist, _, _ = sub8_utils.odometry_to_numpy(msg)
        position, orientation = pose
        linear, angular = twist
        self.cur_state_history.append(np.hstack((position, linear)))

        state_section = self.cur_state_history[::-1][:self.T]

    def plot(self, close_after=None):
        '''TODO:
            Visualize attitude (reduced is fine)'''
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for state_history in self.all_histories:
            # I am not using quivers be
            ax.plot(
                state_history[:, 0],
                state_history[:, 1],
                state_history[:, 2]
            )

        ax.set_xlim(-11., 11.)
        ax.set_ylim(-11., 11.)
        ax.set_zlim(-11., 0.)

        # Don't close with timeout if you don't want to close
        if close_after is not None:
            # Creating a timer object and setting an interval of $close_after seconds
            timer = fig.canvas.new_timer(interval=close_after * 1000)
            timer.add_callback(lambda: plt.close())
            timer.start()
        plt.show()

    def analyze_stability(self):
        '''
        To provide more information that simple steady-state error, this provides some quantitative
            information on the error bounds of the controller post convergence

        We are trying to compute the error envelope

           ^ Error (*'s are points)
           #
           #
           #
           #**
           #  *
           #    *
         _ #______*________________(Max envelope of convergence error)
         | #         *    *     *
         | #___________*____*_____*_(Min error)
         ^ #
           ############################
                      Time --->

        TODO:
            - Analyze other behaviors
            - Provide more useful feedback beyond the size of the convergence ball
        '''
        envelopes = []
        for state_history in self.all_histories:
            errors = state_history[:, :3] - self.target_state[:3]
            min_error = np.array([np.inf, np.inf, np.inf])
            max_envelope = np.array([-np.inf, -np.inf, -np.inf])

            for error in errors:
                abs_error = np.abs(error)
                less_error = abs_error < min_error  # Vector comparison
                min_error[less_error] = abs_error[less_error]
                if not any(less_error):
                    # We've hit our minimum, now let's try to find an envelope
                    more_error = abs_error > max_envelope
                    max_envelope[more_error] = abs_error[more_error]

            envelopes.append((min_error, max_envelope))

        envelopes = np.vstack(envelopes)
        averages = np.average(envelopes, axis=0)
        return averages

    def run_loop(self):
        runs = 0
        while (not rospy.is_shutdown()) and (runs < self.num_runs):
            self.set_pose(
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Vector3(
                        (np.random.random() * 15) - 7.5,
                        (np.random.random() * 15) - 7.5,
                        (np.random.random() * 7) - 9  # Z must be <= 2!
                    )
                )
            )
            self.set_target((0.0, 0.0, -5.0))
            self.cur_state_history = []

            rospy.sleep(self.time_limit)
            self.all_histories.append(np.vstack(self.cur_state_history))
            runs += 1

        if self.do_plot:
            self.plot()
