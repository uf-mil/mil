#pragma once

#include <vector>

#include <ros/ros.h>

#include <mil_tools/mil_tools.hpp>
#include <mil_vision_lib/cv_tools.hpp>
#include <mil_vision_lib/image_acquisition/ros_camera_stream.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

using ROSCameraStream_Vec3 = mil_vision::ROSCameraStream<cv::Vec3b>;

class StereoBase
{
public:
  StereoBase();

  /**
  * Check if left+right cameras are publishing and are in sync
  * @see sync_thresh_
  */
  bool is_stereo_coherent();

  /**
  * A pure virtual method to segment left/right camera images
  * @param image left and right ROSimagestream will pass to this method
  * @see get_3d_feature_points()
  * @see ROSCameraStream
  * @return a vector of points that define the shape in 2d
  */
  virtual std::vector<cv::Point> get_2d_feature_points(cv::Mat image) = 0;

  /**
  * Use stereo intinsics and triangulation to map 2d features to 3d in stereo frame
  * @param max_z filter points that are greater than the given z value
  * @see get_2d_feature_points()
  * @see mil_vision::triangulate_Linear_LS
  * @return vector of points in 3d space or, if failed, a 0-sized vector
  */
  std::unique_ptr<std::vector<Eigen::Vector3d>> get_3d_feature_points(int max_z = 5);

  /**
  * Use 3d points to estimate a normal and a center point and return a pose
  * @param feature_pts_3d a vector of 3d points (currently only supports 4 points)
  * @param z_vector_min the minimum value the z-component of the normal vector should be
  * @see best_fit_plane_standard()
  * @return The translation and rotation with respect to [1,0,0] vector
  */
  std::unique_ptr<Eigen::Affine3d> get_3d_pose(std::vector<Eigen::Vector3d> feature_pts_3d, float z_vector_min = 0.5);

protected:
  std::unique_ptr<ROSCameraStream_Vec3> left_cam_stream_;
  std::unique_ptr<ROSCameraStream_Vec3> right_cam_stream_;

  /**
  * how often will image processing occur
  * also used as dt in kalman filter, defaults to 10 times per second
  * @see init_kalman_filter
  */
  double refresh_rate_;

  /**
  * maximum time difference between the left and right camera time stamps
  * @see is_stereo_coherent()
  */
  double sync_thresh_;

  /**
  * should the node run image processing
  */
  bool active_;

  /**
  * the number of states in the kalman filter
  * default to 18 which includes x,y,z and euler angles, and their respective 1st and 2nd dervatives
  */
  int n_states_;

  /**
  * number of measurments
  * default to 6 which contains x,y,z, and euler angles
  */
  int n_measurements_;

  /**
  * openCV's implementation of the kalman filter
  */
  cv::KalmanFilter k_filter_;

  /**
  * initializes/clears the kalman filter
  * @see k_filter_
  */
  void init_kalman_filter();

  /**
  * Updates the kalman filter and returns the predicted pose
  * @param pose The estimated 3d pose
  * @return The predicted pose
  */
  Eigen::Affine3d update_kalman_filter(const Eigen::Affine3d &pose);

private:
  /**
  * given two sets of points, finds a set of pair with shortest distances
  * @param features_l left camera feature points in 2D
  * @param features_r right camera feature points in 2D
  * @see get_3d_feature_points()
  * @return an vector of indices such that features_l[i] = features_r[correspondence[i]]
  */
  std::vector<int> shortest_pair_stereo_matching(const std::vector<cv::Point> &features_l,
                                                 const std::vector<cv::Point> &features_r, int y_axis_diff_thresh = 0);

  /**
  * Finds the plane of 4 points in standard form
  * @param feature_pts_3d a set of 3D points (currently supports only 4)
  * @return a vector that contains plane constans in standard form (A, B, C, D)
  */
  std::vector<double> best_fit_plane_standard(const std::vector<Eigen::Vector3d> &feature_pts_3d);

  /**
  * helper function for update_kalman_filter that converts to a cv::Mat
  * @param pose Estimated 3d pose
  * @return pose in a cv::Mat form of size (n_measurments_ * 1)
  */
  cv::Mat get_measurement_as_cv_mat(const Eigen::Affine3d &pose);
};