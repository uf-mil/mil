#include <navigator_vision_lib/stereomodelfitter.h>

StereoModelFitter::StereoModelFitter(PerceptionModel model) : model(model) {}

using namespace std;
using namespace cv;

void StereoModelFitter::denoise_images(Mat &l_diffused, Mat &r_diffused,
                                       int diffusion_time,
                                       Mat current_image_left,
                                       Mat current_image_right) {
  Mat diffusion_size_left, diffusion_size_right;
  resize(current_image_left, diffusion_size_left, Size(0, 0), image_proc_scale,
         image_proc_scale);
  resize(current_image_right, diffusion_size_right, Size(0, 0),
         image_proc_scale, image_proc_scale);
  cvtColor(diffusion_size_left, diffusion_size_left, CV_BGR2GRAY);
  cvtColor(diffusion_size_right, diffusion_size_right, CV_BGR2GRAY);
  boost::thread diffusion_L(nav::anisotropic_diffusion,
                            boost::cref(diffusion_size_left),
                            boost::ref(l_diffused), diffusion_time);
  boost::thread diffusion_R(nav::anisotropic_diffusion,
                            boost::cref(diffusion_size_right),
                            boost::ref(r_diffused), diffusion_time);
  diffusion_L.join();
  diffusion_R.join();
}

void StereoModelFitter::extract_features(vector<Point> &features, Mat &image,
                                         int max_corners, int block_size,
                                         double quality_level,
                                         double min_distance) {
  goodFeaturesToTrack(image, features, max_corners, quality_level, min_distance,
                      Mat(), block_size, true, .001);
}

cv::Mat getROI(cv::Mat image, int point_x, int point_y, int size) {
  int width = size;
  int height = size;
  int x = point_x;
  int y = point_y;

  if (point_x - width / 2 < 0)
    x = 0;
  else
    x = point_x - width / 2;
  if (point_y - height / 2 < 0)
    y = 0;
  else
    y = point_y - height / 2;

  if (x + width >= image.cols)
    width = image.cols - x - 1;
  if (y + height >= image.rows)
    height = image.rows - y - 1;
  cv::Rect roi(x, y, width, height);
  return image(roi);
}

void StereoModelFitter::get_corresponding_pairs(
    cv::Mat frame_l, cv::Mat frame_r, vector<Point> features_l,
    vector<Point> features_r, vector<Point> &features_l_out,
    vector<Point> &features_r_out, int picture_width) {
  double curr_min_dist, curr_min_dist_patch, xdiff, ydiff, dist;
  int curr_min_dist_idx;
  int y_diff_thresh = picture_width * 0.02;
  int x_diff_thresh = picture_width * 0.15;
  for (size_t i = 0; i < features_l.size(); i++) {
    curr_min_dist_idx = -1;
    curr_min_dist = 1E6;
    curr_min_dist_patch = 1E6;
    for (size_t j = 0; j < features_r.size(); j++) {
      ydiff = features_l[i].y - features_r[j].y;
      xdiff = features_l[i].x - features_r[j].x;
      // cout << "\t   ydiff: " << ydiff << endl;
      if (abs(ydiff) > y_diff_thresh)
        continue;
      if (abs(xdiff) > x_diff_thresh)
        continue;
      if (xdiff == 0 && ydiff == 0)
        continue;

      int result_cols = 1;
      int result_rows = 1;

      cv::Mat result;
      result.create(result_rows, result_cols, CV_32FC1);
      cv::Mat search_image_left =
          getROI(frame_l, features_l[i].x, features_l[i].y, 7);
      cv::Mat search_image_right =
          getROI(frame_r, features_r[j].x, features_r[j].y, 7);
      int match_method = cv::TM_SQDIFF;
      cv::matchTemplate(search_image_left, search_image_right, result,
                        match_method);
      if (result.at<float>(0, 0) < curr_min_dist_patch) {
        curr_min_dist_patch = result.at<float>(0, 0);
        curr_min_dist_idx = j;
      }
    }

    if (curr_min_dist_idx != -1) {
      features_l_out.push_back(features_l[i]);
      features_r_out.push_back(features_r[curr_min_dist_idx]);
    }
  }
}

void StereoModelFitter::calculate_3D_reconstruction(
    vector<Eigen::Vector3d> &feature_pts_3d, vector<Point> features_l,
    vector<Point> features_r) {

  Eigen::Vector3d pt_3D;
  double reset_scaling = 1 / image_proc_scale;
  for (size_t i = 0; i < features_l.size(); i++) {
    Point2d pt_L = features_l[i];
    Point2d pt_R = features_r[i];

    // Undo the effects of working with coordinates from scaled images
    pt_L = pt_L * reset_scaling;
    pt_R = pt_R * reset_scaling;
    Matx31d pt_L_hom(pt_L.x, pt_L.y, 1);
    Matx31d pt_R_hom(pt_R.x, pt_R.y, 1);
    Mat X_hom = nav::triangulate_Linear_LS(
        Mat(*left_cam_mat), Mat(*right_cam_mat), Mat(pt_L_hom), Mat(pt_R_hom));
    X_hom = X_hom / X_hom.at<double>(3, 0);
    pt_3D << X_hom.at<double>(0, 0), X_hom.at<double>(1, 0),
        X_hom.at<double>(2, 0);
    feature_pts_3d.push_back(pt_3D);
  }
}

bool StereoModelFitter::check_for_model(
    vector<Eigen::Vector3d> feature_pts_3d, vector<cv::Point> left_points_2d,
    vector<Eigen::Vector3d> &correct_model,
    std::vector<cv::Point> &model_position_2d) {
  vector<int> debug_vec;
  //    string bin;
  //    cin >> bin;
  //    if(bin != "done"){
  //      debug_vec  = split(bin);
  //    }

  decision_tree(feature_pts_3d, left_points_2d, -1, model.min_points, debug_vec,
                false);
  bool got_model = model.get_model(correct_model, model_position_2d,
                                   *current_image_left, *left_cam_mat);
  model.clear();
  return got_model;
}

void StereoModelFitter::decision_tree(vector<Eigen::Vector3d> feature_pts_3d,
                                      vector<cv::Point> left_points_2d,
                                      int curr, int remaining,
                                      vector<int> debug_points, bool debug) {
  int total = feature_pts_3d.size();

  // If the current model has the right amount of points or we have no more
  // combinations left to check (the second is condition is unecessary, but for
  // safety)
  if (!model.complete() || remaining > 0) {
    for (int i = curr + 1; i <= total - remaining; ++i) {
      Eigen::Vector3d point = feature_pts_3d[i];
      cv::Point point2d = left_points_2d[i];
      bool val = false;
      int a = model.current_points.size();
      if (((std::find(debug_points.begin(), debug_points.end(), i) !=
            debug_points.end()) &&
           a == 0) ||
          debug) {
        debug = true;
        val = model.check_point(point, point2d, *current_image_left,
                                *left_cam_mat, true);
      } else {
        val = model.check_point(point, point2d, *current_image_left,
                                *left_cam_mat, false);
        debug = false;
      }

      // If that point was correct, keep checking the rest of the points
      if (val) {
        decision_tree(feature_pts_3d, left_points_2d, i, remaining - 1,
                      debug_points, debug);
      }
      // Remove that point from the model so that other points can be checked
      model.remove_point(point, point2d);
    }
  }
}

bool StereoModelFitter::determine_model_position(
    vector<Eigen::Vector3d> &model_position,
    vector<cv::Point> &model_position_2d, int max_corners, int block_size,
    double min_distance, double image_proc_scale, int diffusion_time,
    Mat current_image_left, Mat current_image_right, Matx34d left_cam_mat,
    Matx34d right_cam_mat) {
  this->image_proc_scale = image_proc_scale;
  this->left_cam_mat = &left_cam_mat;
  this->right_cam_mat = &right_cam_mat;
  this->current_image_left = &current_image_left;
  this->current_image_right = &current_image_right;

  Mat l_diffused, r_diffused;
  denoise_images(l_diffused, r_diffused, diffusion_time, current_image_left,
                 current_image_right);
  vector<Point> features_l, features_r;
  Mat l_diffused_draw = l_diffused.clone();
  Mat r_diffused_draw = r_diffused.clone();
  double quality_level = 0.05;
  extract_features(features_r, r_diffused_draw, max_corners, block_size,
                   quality_level, min_distance);
  extract_features(features_l, l_diffused_draw, max_corners, block_size,
                   quality_level, min_distance);

  for (cv::Point p : features_l) {
    circle(l_diffused_draw, p, 5, cv::Scalar(0, 0, 0), -1);
  }
  for (cv::Point p : features_r) {
    circle(r_diffused_draw, p, 5, cv::Scalar(0, 0, 0), -1);
  }

  Mat l_diffused_draw1 = l_diffused.clone();
  Mat r_diffused_draw1 = r_diffused.clone();

  vector<Point> points_l, points_r;
  get_corresponding_pairs(l_diffused, r_diffused, features_l, features_r,
                          points_l, points_r, l_diffused.rows);

  for (size_t i = 0; i < points_l.size(); i++) {
    cv::Point p = points_l[i];
    circle(l_diffused_draw1, p, 5, cv::Scalar(0, 0, 0), -1);
    stringstream label;
    label << i;
    putText(l_diffused_draw1, label.str(), p, FONT_HERSHEY_SIMPLEX,
            0.0015 * l_diffused_draw1.rows, Scalar(0, 0, 0), 2);
  }
  for (size_t i = 0; i < points_l.size(); i++) {
    cv::Point p = points_r[i];
    circle(r_diffused_draw1, p, 5, cv::Scalar(0, 0, 0), -1);
    stringstream label;
    label << i;
    putText(r_diffused_draw1, label.str(), p, FONT_HERSHEY_SIMPLEX,
            0.0015 * r_diffused_draw1.rows, Scalar(0, 0, 0), 2);
  }

  vector<Eigen::Vector3d> feature_pts_3d;
  calculate_3D_reconstruction(feature_pts_3d, points_l, points_r);

  std::cout << feature_pts_3d.size() << std::endl;

  vector<Eigen::Vector3d> correct_model;

  visualize_points(feature_pts_3d, current_image_left);

  vector<cv::Point> left_points;
  double reset_scaling = 1 / image_proc_scale;
  for (cv::Point point : points_l) {
    Point2d pt_L = point;
    pt_L = pt_L * reset_scaling;
    left_points.push_back(pt_L);
  }

  bool got_model = check_for_model(feature_pts_3d, points_l, correct_model,
                                   model_position_2d);
  model_position = correct_model;
  return got_model;
}

void StereoModelFitter::visualize_points(vector<Eigen::Vector3d> feature_pts_3d,
                                         Mat &img_left) {
  cv::Mat current_image_left = img_left.clone();
  for (size_t i = 0; i < feature_pts_3d.size(); i++) {
    Eigen::Vector3d pt = feature_pts_3d[i];
    Matx41d position_hom(pt(0), pt(1), pt(2), 1);
    Matx31d pt_L_2d_hom = *left_cam_mat * position_hom;
    Point2d L_center2d(pt_L_2d_hom(0) / pt_L_2d_hom(2),
                       pt_L_2d_hom(1) / pt_L_2d_hom(2));
    Scalar color(255, 0, 255);
    stringstream label;
    label << i;
    circle(current_image_left, L_center2d, 5, color, -1);
    putText(current_image_left, label.str(), L_center2d, FONT_HERSHEY_SIMPLEX,
            0.0015 * current_image_left.rows, Scalar(0, 0, 0), 2);
  }
  debug_image_3dpoints.publish(
      nav::convert_to_ros_msg("bgr8", current_image_left));
  ros::spinOnce();
}

std::vector<int> split(string str) {
  std::vector<int> vect;
  std::stringstream ss(str);
  int i;
  while (ss >> i) {
    vect.push_back(i);
    if (ss.peek() == ',')
      ss.ignore();
  }
  return vect;
}
