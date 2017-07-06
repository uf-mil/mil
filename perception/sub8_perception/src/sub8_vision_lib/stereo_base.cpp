#include <sub8_vision_lib/stereo_base.hpp>

StereoBase::StereoBase()
{
  n_states_ = 18;
  n_measurements_ = 6;
  refresh_rate_ = 10;
  init_kalman_filter();
}

bool StereoBase::is_stereo_coherent()
{
  if (!left_cam_stream_->ok() || !right_cam_stream_->ok())
  {
    ROS_WARN("Start Gate Detector: not getting images.");
    return false;
  }

  if (left_cam_stream_->size() < 1 || right_cam_stream_->size() < 1)
  {
    ROS_WARN("Calling too soon -- no images ready");
    return false;
  }

  // Enforce approximate image synchronization
  double left_stamp, right_stamp;
  left_stamp = (*left_cam_stream_)[0]->stamp().toSec();
  right_stamp = (*right_cam_stream_)[0]->stamp().toSec();
  double sync_error = fabs(left_stamp - right_stamp);
  if (sync_error > sync_thresh_)
  {
    ROS_WARN("Left and right not synchronized");
    return false;
  }
  return true;
}

std::unique_ptr<std::vector<Eigen::Vector3d>> StereoBase::get_3d_feature_points(int max_z)
{
  std::vector<cv::Point> features_l, features_r;
  features_l = get_2d_feature_points((*left_cam_stream_)[0]->image());
  features_r = get_2d_feature_points((*right_cam_stream_)[0]->image());

  std::vector<int> correspondence_pair_idxs =
      shortest_pair_stereo_matching(features_l, features_r, left_cam_stream_->rows() * 0.02);

  // Check if we have any undefined correspondence pairs
  if (std::count(correspondence_pair_idxs.begin(), correspondence_pair_idxs.end(), -1) != 0)
    return nullptr;

  cv::Matx34d left_cam_mat = (*left_cam_stream_)[0]->getCameraModelPtr()->fullProjectionMatrix();
  cv::Matx34d right_cam_mat = (*right_cam_stream_)[0]->getCameraModelPtr()->fullProjectionMatrix();

  // Calculate 3D stereo reconstructions
  std::vector<Eigen::Vector3d> feature_pts_3d;
  Eigen::Vector3d pt_3D;
  for (size_t i = 0; i < correspondence_pair_idxs.size(); i++)
  {
    if (correspondence_pair_idxs[i] == -1)
      continue;
    cv::Point2d pt_L = features_l[i];
    cv::Point2d pt_R = features_r[correspondence_pair_idxs[i]];
    cv::Matx31d pt_L_hom(pt_L.x, pt_L.y, 1);
    cv::Matx31d pt_R_hom(pt_R.x, pt_R.y, 1);
    cv::Mat X_hom = mil_vision::triangulate_Linear_LS(cv::Mat(left_cam_mat), cv::Mat(right_cam_mat), cv::Mat(pt_L_hom),
                                                      cv::Mat(pt_R_hom));
    X_hom = X_hom / X_hom.at<double>(3, 0);
    pt_3D << X_hom.at<double>(0, 0), X_hom.at<double>(1, 0), X_hom.at<double>(2, 0);
    if (pt_3D(2) < 0)
      return nullptr;
    if (pt_3D(2) > max_z)
      return nullptr;
    feature_pts_3d.push_back(pt_3D);
  }
  return std::unique_ptr<std::vector<Eigen::Vector3d>>(new std::vector<Eigen::Vector3d>(feature_pts_3d));
}

std::unique_ptr<Eigen::Affine3d> StereoBase::get_3d_pose(std::vector<Eigen::Vector3d> feature_pts_3d,
                                                         float z_vector_min)
{
  // Will cause problems if not 4 points
  if (feature_pts_3d.size() != 4)
    return nullptr;
  // Ignore if the volume of the 4 points is not close to 0
  Eigen::Matrix3d matrix_of_vectors;
  {
    Eigen::Vector3d pt0 = feature_pts_3d[0];
    Eigen::Vector3d pt1 = feature_pts_3d[1];
    Eigen::Vector3d pt2 = feature_pts_3d[2];
    Eigen::Vector3d pt3 = feature_pts_3d[3];

    matrix_of_vectors.col(0) = pt1 - pt0;
    matrix_of_vectors.col(1) = pt2 - pt0;
    matrix_of_vectors.col(2) = pt3 - pt0;
  }
  // Check if the 4 points form a plane
  if (abs(matrix_of_vectors.determinant()) * 100 > 30)
    return nullptr;

  std::vector<double> plane_constants = best_fit_plane_standard(feature_pts_3d);

  // Project points to best fit plane
  std::vector<Eigen::Vector3d> proj_pts;
  Eigen::Vector3d plane_unit_normal;
  plane_unit_normal << plane_constants[0], plane_constants[1], plane_constants[2];
  plane_unit_normal = plane_unit_normal / plane_unit_normal.norm();

  // Reject if z is too far off
  if (fabs(plane_unit_normal(2, 0)) < z_vector_min)
    return nullptr;

  Eigen::Vector3d pt_on_plane;
  pt_on_plane << 0, 0, -plane_constants[3] / plane_constants[2];
  for (uint8_t pt_idx = 0; pt_idx < feature_pts_3d.size(); ++pt_idx)
  {
    Eigen::Vector3d pt = feature_pts_3d[pt_idx];
    Eigen::Vector3d plane_to_pt_vec = pt - pt_on_plane;
    Eigen::Vector3d plane_to_pt_proj_normal = plane_to_pt_vec.dot(plane_unit_normal) * plane_unit_normal;
    Eigen::Vector3d corr_pt = pt - plane_to_pt_proj_normal;
    proj_pts.push_back(corr_pt);
  }

  // Flip the vector to point towards camera if necessary
  plane_unit_normal = plane_unit_normal(2, 0) < 0 ? plane_unit_normal : -plane_unit_normal;

  Eigen::Vector3d center_pt;
  for (auto p : proj_pts)
  {
    center_pt(0, 0) += p[0];
    center_pt(1, 0) += p[1];
    center_pt(2, 0) += p[2];
  }
  center_pt(0, 0) /= proj_pts.size();
  center_pt(1, 0) /= proj_pts.size();
  center_pt(2, 0) /= proj_pts.size();

  Eigen::Quaterniond orientation = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d(1, 0, 0), plane_unit_normal);

  return std::unique_ptr<Eigen::Affine3d>(
      new Eigen::Affine3d(Eigen::Translation3d(center_pt(0, 0), center_pt(1, 0), center_pt(2, 0)) * orientation));
}

std::vector<int> StereoBase::shortest_pair_stereo_matching(const std::vector<cv::Point> &features_l,
                                                           const std::vector<cv::Point> &features_r,
                                                           int y_axis_diff_thresh)
{
  std::vector<int> correspondence_pair_idxs;
  double curr_min_dist, xdiff, ydiff, dist;
  int curr_min_dist_idx;
  for (size_t i = 0; i < features_l.size(); i++)
  {
    curr_min_dist_idx = -1;
    curr_min_dist = 1E6;
    for (size_t j = 0; j < features_r.size(); j++)
    {
      ydiff = features_l[i].y - features_r[j].y;
      if (abs(ydiff) > y_axis_diff_thresh)
        continue;
      xdiff = features_l[i].x - features_r[j].x;

      dist = sqrt(xdiff * xdiff + ydiff * ydiff);
      if (dist < curr_min_dist)
      {
        curr_min_dist = dist;
        curr_min_dist_idx = j;
      }
    }
    correspondence_pair_idxs.push_back(curr_min_dist_idx);
  }
  return correspondence_pair_idxs;
}

std::vector<double> StereoBase::best_fit_plane_standard(const std::vector<Eigen::Vector3d> &feature_pts_3d)
{
  // Calculate best fit plane
  Eigen::Matrix<double, 4, 3> A;
  Eigen::Matrix<double, 4, 1> b_vec;
  A << feature_pts_3d[0][0], feature_pts_3d[0][1], feature_pts_3d[0][2], feature_pts_3d[1][0], feature_pts_3d[1][1],
      feature_pts_3d[1][2], feature_pts_3d[2][0], feature_pts_3d[2][1], feature_pts_3d[2][2], feature_pts_3d[3][0],
      feature_pts_3d[3][1], feature_pts_3d[3][2];
  b_vec << 1, 1, 1, 1;
  Eigen::Matrix<double, 3, 1> x = A.colPivHouseholderQr().solve(b_vec);
  std::vector<double> plane_constants(4);
  plane_constants[0] = 1;            // A
  plane_constants[1] = x[1] / x[0];  // B
  plane_constants[2] = x[2] / x[0];  // C
  plane_constants[3] = -1 / x[0];    // D
  return plane_constants;
}

void StereoBase::init_kalman_filter()
{
  k_filter_.init(n_states_, n_measurements_, 0, CV_64F);
  cv::setIdentity(k_filter_.processNoiseCov, cv::Scalar::all(1e-5));
  // How much to trust measurements
  cv::setIdentity(k_filter_.measurementNoiseCov, cv::Scalar::all(0.005));
  cv::setIdentity(k_filter_.errorCovPost, cv::Scalar::all(1));

  // position
  k_filter_.transitionMatrix.at<double>(0, 3) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(1, 4) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(2, 5) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(3, 6) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(4, 7) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(5, 8) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(0, 6) = 0.5 * pow(1 / refresh_rate_, 2);
  k_filter_.transitionMatrix.at<double>(1, 7) = 0.5 * pow(1 / refresh_rate_, 2);
  k_filter_.transitionMatrix.at<double>(2, 8) = 0.5 * pow(1 / refresh_rate_, 2);

  // orientation
  k_filter_.transitionMatrix.at<double>(9, 12) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(10, 13) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(11, 14) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(12, 15) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(13, 16) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(14, 17) = 1 / refresh_rate_;
  k_filter_.transitionMatrix.at<double>(9, 15) = 0.5 * pow(1 / refresh_rate_, 2);
  k_filter_.transitionMatrix.at<double>(10, 16) = 0.5 * pow(1 / refresh_rate_, 2);
  k_filter_.transitionMatrix.at<double>(11, 17) = 0.5 * pow(1 / refresh_rate_, 2);

  // x
  k_filter_.measurementMatrix.at<double>(0, 0) = 1;
  // y
  k_filter_.measurementMatrix.at<double>(1, 1) = 1;
  // z
  k_filter_.measurementMatrix.at<double>(2, 2) = 1;
  // roll
  k_filter_.measurementMatrix.at<double>(3, 9) = 1;
  // pitch
  k_filter_.measurementMatrix.at<double>(4, 10) = 1;
  // yaw
  k_filter_.measurementMatrix.at<double>(5, 11) = 1;
}

cv::Mat StereoBase::get_measurement_as_cv_mat(const Eigen::Affine3d &pose)
{
  cv::Mat measurement(n_measurements_, 1, CV_64F);
  Eigen::Vector3d euler = pose.rotation().eulerAngles(0, 1, 2);
  measurement.at<double>(0) = pose.translation().x();
  measurement.at<double>(1) = pose.translation().y();
  measurement.at<double>(2) = pose.translation().z();
  measurement.at<double>(3) = euler(0, 0);
  measurement.at<double>(4) = euler(1, 0);
  measurement.at<double>(5) = euler(2, 0);
  return measurement;
}

Eigen::Affine3d StereoBase::update_kalman_filter(const Eigen::Affine3d &pose)
{
  cv::Mat prediction = k_filter_.predict();
  cv::Mat measurement = get_measurement_as_cv_mat(pose);

  cv::Mat estimated = k_filter_.correct(measurement);

  Eigen::Vector3d euler;
  euler(0, 0) = estimated.at<double>(9);
  euler(1, 0) = estimated.at<double>(10);
  euler(2, 0) = estimated.at<double>(11);

  Eigen::AngleAxisd rollAngle(euler(0, 0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(euler(1, 0), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(euler(2, 0), Eigen::Vector3d::UnitZ());

  Eigen::Translation3d translation(estimated.at<double>(0), estimated.at<double>(1), estimated.at<double>(2));
  Eigen::Quaterniond orientation = rollAngle * yawAngle * pitchAngle;
  return Eigen::Affine3d(translation * orientation);
}
