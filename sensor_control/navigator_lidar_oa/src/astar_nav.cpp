AStar astar(ROI_SIZE_METERS / VOXEL_SIZE_METERS);
geometry_msgs::Pose boatPose_enu;
geometry_msgs::Twist boatTwist_enu;
uf_common::PoseTwistStamped waypoint_enu, carrot_enu;

typedef actionlib::SimpleActionServer<uf_common::MoveToAction> MOVE_TO_SERVER;
MOVE_TO_SERVER *actionServerPtr;

void actionExecute(const uf_common::MoveToGoalConstPtr &goal) {
  // Grab new goal from actionserver
  ROS_INFO("LIDAR: Following new goal from action server!");
  waypoint_enu.posetwist = goal->posetwist;
}

void cb_velodyne(const sensor_msgs::PointCloud2ConstPtr &pcloud) {
  // Set the carrot as the boat's current position and orientation - this is the backup for the boat not to move if
  // Astar fails
  carrot_enu.posetwist.pose.position = boatPose_enu.position;
  carrot_enu.posetwist.pose.orientation = boatPose_enu.orientation;
  carrot_enu.posetwist.twist.linear.x = 0;
  carrot_enu.posetwist.twist.linear.y = 0;
  carrot_enu.posetwist.twist.linear.z = 0;
  carrot_enu.posetwist.twist.angular.x = 0;
  carrot_enu.posetwist.twist.angular.y = 0;
  carrot_enu.posetwist.twist.angular.z = 0;

  // If the action server is active, run Astar
  if (actionServerPtr->isActive()) {
    ROS_INFO("LIDAR: Action server has an active goal to follow!");

    // Fake waypoint instead of using action server request
    // waypoint_enu.posetwist.pose.position.x = 25;
    // waypoint_enu.posetwist.pose.position.y = -25;
    // waypoint_enu.posetwist.pose.position.z = 5;

    // Convert waypoint from enu frame to ROI around lidar
    waypoint_ogrid.x =
        (int)((waypoint_enu.posetwist.pose.position.x - lidarpos.x) / VOXEL_SIZE_METERS + ogrid.ROI_SIZE / 2);
    waypoint_ogrid.y =
        (int)((waypoint_enu.posetwist.pose.position.y - lidarpos.y) / VOXEL_SIZE_METERS + ogrid.ROI_SIZE / 2);
    waypoint_ogrid.z = waypoint_enu.posetwist.pose.position.z;

    // Force waypoint to fit on ROI
    if (waypoint_ogrid.x >= ogrid.ROI_SIZE) {
      waypoint_ogrid.x = ogrid.ROI_SIZE - 1;
    }
    if (waypoint_ogrid.x < 0) {
      waypoint_ogrid.x = 0;
    }
    if (waypoint_ogrid.y >= ogrid.ROI_SIZE) {
      waypoint_ogrid.y = ogrid.ROI_SIZE - 1;
    }
    if (waypoint_ogrid.y < 0) {
      waypoint_ogrid.y = 0;
    }

    // Inflate waypoint on grid for easier visuals
    for (int ii = -2; ii <= 2; ++ii) {
      for (int jj = -2; jj <= 2; ++jj) {
        ogrid.ogridMap[(waypoint_ogrid.y + ii) * ogrid.ROI_SIZE + waypoint_ogrid.x + jj] = 25;
      }
    }

    // Setup Astar
    astar.setMap(ogrid.ogridBinary);
    astar.setFinish(waypoint_ogrid.x, waypoint_ogrid.y);

    // Mark starting square on map
    ogrid.ogridMap[(astar.startNode.y) * ogrid.ROI_SIZE + astar.startNode.x] = 25;

    // Run Astar
    auto solution = astar.run();

    // Determine the boats current rotation in the plane of the water
    double boatRotAngle = atan2(Eigen::Vector3d(boatPose_enu.orientation.y, boatPose_enu.orientation.x,
                                                boatPose_enu.orientation.z).norm(),
                                boatPose_enu.orientation.w) *
                          2 * 180 / 3.14159;
    ROS_INFO_STREAM("LIDAR | Boat Rotation angle: " << boatRotAngle);

    // If we are close to the goal, make that the waypoint
    if (solution.size() > 1 && solution.size() <= 5) {
      carrot_enu.posetwist = waypoint_enu.posetwist;
      // actionServerPtr->setSucceeded();
    }

    // If the Astar found a path with several steps to the goal, pick a spot a few iterations ahead of the boat
    const int STEPS_ALONG_ASTAR = 20;
    Eigen::Vector3d desHeading(0, 0, 0);
    double desBoatRotAngle = 0;
    int index = 0;
    for (auto square : solution) {
      // Place path on map
      ogrid.ogridMap[square.second * ogrid.ROI_SIZE + square.first] = 0;

      // At some arbitrary distance away, make this the desired waypoint
      if (index == STEPS_ALONG_ASTAR) {
        // Determine heading and corresponding angle
        desHeading(0) = square.first - ogrid.ROI_SIZE / 2;
        desHeading(1) = square.second - ogrid.ROI_SIZE / 2;
        desHeading(2) = 0;
        desHeading.normalize();
        desBoatRotAngle = atan2(desHeading(1), desHeading(0));
        ROS_INFO_STREAM("LIDAR | Desired Heading: " << desHeading(0) << "," << desHeading(1) << " -> "
                                                    << desBoatRotAngle * 180 / 3.14159);

        // Update carrot in the enu
        // We only let the boat translate if we are pointing close to the right direction, otherwise just rotate
        if (fabs(desBoatRotAngle - boatRotAngle) < 15) {
          carrot_enu.posetwist.pose.position.x = (square.first - ogrid.ROI_SIZE / 2) * ogrid.VOXEL_SIZE_METERS +
                                                 ogrid.lidarPos.x - ogrid.VOXEL_SIZE_METERS;
          carrot_enu.posetwist.pose.position.y = (square.second - ogrid.ROI_SIZE / 2) * ogrid.VOXEL_SIZE_METERS +
                                                 ogrid.lidarPos.y - ogrid.VOXEL_SIZE_METERS;
          carrot_enu.posetwist.twist.linear.x = desHeading(0) * 1;  // speed of 1 m/s
          carrot_enu.posetwist.twist.linear.y = desHeading(1) * 1;  // speed of 1 m/s
          carrot_enu.posetwist.twist.linear.z = 0;
        }

        // Create quaternion facing in the direction of the carrot
        carrot_enu.posetwist.pose.orientation.w = cos(desBoatRotAngle / 2);
        carrot_enu.posetwist.pose.orientation.x = 0;
        carrot_enu.posetwist.pose.orientation.y = 0;
        carrot_enu.posetwist.pose.orientation.z = 1 * sin(desBoatRotAngle / 2);

        // Make the carrot look bigger than it is in the ogrid
        int ii = 0, jj = 0;
        for (int ii = -2; ii <= 2; ++ii) {
          for (int jj = -2; jj <= 2; ++jj) {
            ogrid.ogridMap[(square.second + ii) * ogrid.ROI_SIZE + square.first + jj] = 75;
          }
        }
      }
      ++index;
    }
    ROS_INFO_STREAM("LIDAR | Carrot orientation: "
                    << carrot_enu.posetwist.pose.orientation.w << "," << carrot_enu.posetwist.pose.orientation.x << ","
                    << carrot_enu.posetwist.pose.orientation.y << "," << carrot_enu.posetwist.pose.orientation.z);
    ROS_INFO_STREAM("LIDAR | Boat orientation: " << boatPose_enu.orientation.w << "," << boatPose_enu.orientation.x
                                                 << "," << boatPose_enu.orientation.y << ","
                                                 << boatPose_enu.orientation.z);
  }
  ROS_INFO_STREAM("LIDAR | Carrot enu: " << carrot_enu.posetwist.pose.position.x << ","
                                         << carrot_enu.posetwist.pose.position.y << ","
                                         << carrot_enu.posetwist.pose.position.z);
  ROS_INFO_STREAM("LIDAR | Boat enu: " << boatPose_enu.position.x << "," << boatPose_enu.position.y << ","
                                       << boatPose_enu.position.z);
  // pubTrajectory.publish(carrot_enu);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update odometry information
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cb_odom(const nav_msgs::OdometryConstPtr &odom) {
  // ROS_INFO("cb_odom...");
  boatPose_enu = odom->pose.pose;
  boatTwist_enu = odom->twist.twist;
}

int main(int argc, char *argv[]) {
  // Ros init
  ros::init(argc, argv, "lidar");
  ros::Time::init();

  // ros::init(argc, argv, "lidar", ros::init_options::AnonymousName);

  // Check that ROS is alive before continuing... After 10 minutes quit!
  ROS_INFO("LIDAR | Checking ROS master is alive...");
  ros::Time timer = ros::Time::now();
  while (!ros::master::check()) {
    if ((ros::Time::now() - timer).toSec() > 600) {
      return -1;
    }
    ros::Duration(0.1).sleep();
  }
  ROS_INFO_STREAM("ROS Master: " << ros::master::getHost());

  // Initialize Astar to use 8 way search method
  astar.setMode(AStar::ASTAR, AStar::EIGHT);

  // Node handler
  ros::NodeHandle nh;

  // Setup action server
  MOVE_TO_SERVER actionServer(nh, "moveto", boost::bind(&actionExecute, _1, &actionServer), false);
  MOVE_TO_SERVER actionServer(nh, "moveto", actionExecute, false);
  actionServerPtr = &actionServer;
  actionServer.start();

  // Subscribe to odom and the velodyne
  ros::Subscriber sub1 = nh.subscribe("/velodyne_points", 1, cb_velodyne);
  ros::Subscriber sub2 = nh.subscribe("/odom", 1, cb_odom);

  // Publish waypoints to controller
  pubTrajectory = nh.advertise<uf_common::PoseTwistStamped>("trajectory", 1);
  pubWaypoint = nh.advertise<PoseStamped>("waypoint", 1);  // Do we need this?

  // Give control to ROS
  ros::spin();

  return 0;
}
