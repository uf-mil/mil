#include <sub8_perception/start_gate.hpp>
Sub8StartGateDetector::Sub8StartGateDetector() : nh("~"), image_transport(nh), active(true), sync_thresh(0.5), kf(9, 3, 0)
{
  std::string img_topic_left_default = "/camera/front/left/image_rect_color";
  std::string img_topic_right_default = "/camera/front/right/image_rect_color";

  std::string left = nh.param<std::string>("/start_gate_vision/input_left", img_topic_left_default);
  std::string right = nh.param<std::string>("/start_gate_vision/input_right", img_topic_right_default);
  left_image_sub = image_transport.subscribeCamera(left, 10, &Sub8StartGateDetector::left_image_callback, this);
  right_image_sub = image_transport.subscribeCamera(right, 10, &Sub8StartGateDetector::right_image_callback, this);

   marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
   center_gate_pub = nh.advertise<geometry_msgs::Point>("center", 1, true);
   normal_gate_pub = nh.advertise<geometry_msgs::Vector3>("normal", 1, true);

   debug_image_pub_left = image_transport.advertise("left", 1, true);
   debug_image_pub_right = image_transport.advertise("right", 1, true);
   debug_image_pub_canny = image_transport.advertise("canny", 1, true);

  canny_low = nh.param<int>("/start_gate_vision/canny_low", 30);
  canny_ratio = nh.param<int>("/start_gate_vision/canny_ratio", 3.0);
  blur_size = nh.param<int>("/start_gate_vision/blur_size", 7);
  dilate_amount = nh.param<int>("/start_gate_vision/dilate_amount", 3);

  // cv::setIdentity(kf.transitionMatrix);

  // kf.measurementMatrix = cv::Mat::zeroes(9, 3, CV_32F);
  // kf.measurementMatrix.at<float>(0,0) = 1.f;
  // kf.measurementMatrix.at<float>(0,9) = 1.f;
  // kf.measurementMatrix.at<float>(0,18) = 1.f;

  // cv::setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
  // cv::setIdentity(KF.measurementNoiseCov, Scalar::all(10));
  // cv::setIdentity(KF.errorCovPost, Scalar::all(.1));
  run();
}

void Sub8StartGateDetector::left_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                                                   const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
{

  left_mtx.lock();
  left_most_recent.image_msg_ptr = image_msg_ptr;
  left_most_recent.info_msg_ptr = info_msg_ptr;
  left_mtx.unlock();
}

void Sub8StartGateDetector::right_image_callback(const sensor_msgs::ImageConstPtr &image_msg_ptr,
                                                    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
{

  right_mtx.lock();
  right_most_recent.image_msg_ptr = image_msg_ptr;
  right_most_recent.info_msg_ptr = info_msg_ptr;
  right_mtx.unlock();
}

void Sub8StartGateDetector::run()
{
  ros::Rate loop_rate(10);  // process images 10 times per second
  while (ros::ok())
  {
    if (active)
      determine_start_gate_position();
    loop_rate.sleep();
    ros::spinOnce();
  }
  return;
}

void Sub8StartGateDetector::determine_start_gate_position()
{
  // Prevent segfault if service is called before we get valid img_msg_ptr's
  if (left_most_recent.image_msg_ptr == NULL || right_most_recent.image_msg_ptr == NULL)
  {
    ROS_WARN("Start Gate Detector: Image Pointers are NULL.");
    return;
  }

  cv_bridge::CvImagePtr input_bridge;
  cv::Mat current_image_left, current_image_right, processing_size_image_left, processing_size_image_right;

  try
  {
    left_mtx.lock();
    right_mtx.lock();

    // Left Camera
    input_bridge = cv_bridge::toCvCopy(left_most_recent.image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    current_image_left = input_bridge->image;
    left_cam_model.fromCameraInfo(left_most_recent.info_msg_ptr);
    if (current_image_left.channels() != 3)
    {
      ROS_ERROR("The left image topic does not contain a color image.");
      return;
    }

    // Right Camera
    input_bridge = cv_bridge::toCvCopy(right_most_recent.image_msg_ptr, sensor_msgs::image_encodings::BGR8);
    current_image_right = input_bridge->image;
    right_cam_model.fromCameraInfo(right_most_recent.info_msg_ptr);
    if (current_image_right.channels() != 3)
    {
      ROS_ERROR("The right image topic does not contain a color image.");
      return;
    } 
    left_mtx.unlock();
    right_mtx.unlock();
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR("[start_gate] cv_bridge: Failed to convert images");
    left_mtx.unlock();
    right_mtx.unlock();
    return;
  }

  // Enforce approximate image synchronization
  double left_stamp, right_stamp;
  left_stamp = left_most_recent.image_msg_ptr->header.stamp.toSec();
  right_stamp = right_most_recent.image_msg_ptr->header.stamp.toSec();
  double sync_error = fabs(left_stamp - right_stamp);
  if (sync_error > sync_thresh)
  {
    ROS_WARN("Left and right not synchronized");
    return;
  }
  // get_2d_features(current_image_right);
  cv::Mat kernal = cv::Mat::ones(7,7, CV_8U);

  cv::Mat lab_left, lab_right;
  cv::cvtColor(current_image_left, lab_left, cv::COLOR_BGR2Lab );
  cv::cvtColor(current_image_right, lab_right, cv::COLOR_BGR2Lab );

  cv::Mat hsv_left, hsv_right;
  cv::cvtColor(current_image_left, hsv_left, cv::COLOR_BGR2HSV );
  cv::cvtColor(current_image_right, hsv_right, cv::COLOR_BGR2HSV );

  {
    cv::Mat lab_channels[3];
    cv::split(lab_left, lab_channels);
    cv::Mat hsv_channels[3];
    cv::split(hsv_left, hsv_channels);
    cv::bitwise_and(lab_channels[1], hsv_channels[0], lab_left);
  }

  {
    cv::Mat lab_channels[3];
    cv::split(lab_right, lab_channels);
    cv::Mat hsv_channels[3];
    cv::split(hsv_right, hsv_channels);
    cv::bitwise_and(lab_channels[1], hsv_channels[0], lab_right);
  }


  cv::blur(lab_left, lab_left, cv::Size(blur_size, blur_size));
  cv::blur(lab_right, lab_right, cv::Size(blur_size, blur_size));


  cv::Mat canny_left, canny_right;
  cv::Canny(lab_left, canny_left, canny_low, canny_low * 3.0);
  cv::Canny(lab_right, canny_right, canny_low, canny_low * 3.0);


  sensor_msgs::ImagePtr dbg_img_msg_canny = cv_bridge::CvImage(std_msgs::Header(), "mono8", canny_left).toImageMsg();
  debug_image_pub_canny.publish(dbg_img_msg_canny);


  cv::Mat closing_left, closing_right;
  cv::morphologyEx(canny_left, closing_left, cv::MORPH_CLOSE, kernal);
  cv::morphologyEx(canny_right, closing_right, cv::MORPH_CLOSE, kernal);

  cv::Mat dilation_left, dilation_right;
  cv::dilate(closing_left, dilation_left, kernal, cv::Point(), dilate_amount);
  cv::dilate(closing_right, dilation_right, kernal, cv::Point(), dilate_amount);

  std::vector<std::vector<cv::Point>> contours_left, contours_right;
  std::vector<cv::Vec4i> hierarchy_left, hierarchy_right;
  cv::findContours(dilation_left, contours_left, hierarchy_left, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(-1,-1));
  cv::findContours(dilation_right, contours_right, hierarchy_right, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(-1,-1));

/////////////////////
  std::vector<cv::Point> features_l, features_r;

  std::vector<cv::Point> features_l_t, features_r_t;

  for(size_t i = 0; i < contours_left.size(); ++i)
  {
    if(valid_contour(contours_left.at(i)))
    {
      // features_l = contours_left.at(i);
      auto epsilon = 0.01 * cv::arcLength(contours_left.at(i), true);
      // std::vector<cv::Point> approx;
      cv::approxPolyDP(contours_left.at(i), features_l_t, epsilon, true);
      std::vector<std::vector<cv::Point> >v_l;
      v_l.push_back(features_l_t);
      cv::drawContours(current_image_left, v_l, -1, cv::Scalar(200, 0, 0), 3);
      cv::Moments mu = cv::moments(contours_left.at(i), false);
      // features_l.push_back(cv::Point(mu.m10/mu.m00 , mu.m01/mu.m00 ));

    }
  }

  for(size_t i = 0; i < contours_right.size(); ++i)
  {
    if(valid_contour(contours_right.at(i)))
    {

      auto epsilon = 0.01 * cv::arcLength(contours_right.at(i), true);
      // std::vector<cv::Point> approx;
      cv::approxPolyDP(contours_right.at(i), features_r_t, epsilon, true);
      std::vector<std::vector<cv::Point>> v_r;
      v_r.push_back(features_r_t);
      cv::drawContours(current_image_right, v_r, -1, cv::Scalar(0, 0, 200), 3);
      cv::Moments mu = cv::moments(contours_right.at(i), false);
      // features_r.push_back(cv::Point(mu.m10/mu.m00 , mu.m01/mu.m00 ));
    }
  }

//////////////////////////////////////
  if(features_l_t.size() < 5 || features_r_t.size() < 5) return;

  // if(features_l_t.size() != 8 || features_r_t.size() != 8) return;
  
  std::vector<std::vector<uint8_t>> id_comb_l;
  combinations(features_l_t.size(),2,id_comb_l);

  std::sort(id_comb_l.begin(), id_comb_l.end(), 
      [&features_l_t](const std::vector<uint8_t> &a, const std::vector<uint8_t> &b) -> bool 
      {
        auto diffx = features_l_t[a[0]].x - features_l_t[a[1]].x;
        auto diffy = features_l_t[a[0]].y - features_l_t[a[1]].y;
        auto distanceA = std::sqrt(std::pow(diffx, 2) + std::pow(diffy, 2));

        diffx = features_l_t[b[0]].x - features_l_t[b[1]].x;
        diffy = features_l_t[b[0]].y - features_l_t[b[1]].y;
        auto distanceB = std::sqrt(std::pow(diffx, 2) + std::pow(diffy, 2));

        return distanceA < distanceB;
      });

  // for(int i =0; i < id_comb_l.size(); ++i)
  // {
  //   std::cout << (int)id_comb_l[i][0] << " " << (int)id_comb_l[i][1] << " <---> " << features_l_t[id_comb_l[i][0]] << " " << features_l_t[id_comb_l[i][1]] << std::endl;
  // }
  for(int i = 0; i < 4; ++i)
  {
    features_l.push_back((features_l_t[id_comb_l[i][0]] + features_l_t[id_comb_l[i][1]])/2);
    // cv::circle(current_image_left, features_l[i], 10, cv::Scalar(0,255,0), -1);
    // std::cout << "test -->" << features_l[i] << std::endl;

  }
  // cv::Point test =features_l_t[id_comb_l[0][0]] + features_l_t[id_comb_l[0][1]];
  // test /=2;
  // std::cout << "test --> " << test << std::endl;
  // cv::circle(current_image_left, test, 10, cv::Scalar(0,255,0), -1);
  // cv::imshow("sdsd", current_image_left);
  // cv::waitKey(30);

  // std::vector<cv::Point> features_l_t_t;

  // for(int i = 0; i < 2; ++i)
  // {
  //   features_l_t_t.push_back(features_l_t[id_comb_l[i][0]]);
  //   features_l_t_t.push_back(features_l_t[id_comb_l[i][1]]);
  // }

  // features_l.push_back(cv::Point((features_l_t_t[0].x + features_l_t_t[0].x)/2, features_l_t_t[0].y));
  // features_l.push_back(cv::Point((features_l_t_t[1].x + features_l_t_t[1].x)/2, features_l_t_t[1].y));
  // features_l.push_back(cv::Point(features_l_t_t[0].x, (features_l_t_t[0].y + features_l_t_t[0].y)/2));
  // features_l.push_back(cv::Point(features_l_t_t[1].x, (features_l_t_t[1].y + features_l_t_t[1].y)/2));




  std::vector<std::vector<uint8_t>> id_comb_r;
  combinations(features_l_t.size(),2,id_comb_r);

  //Sort approx poly points... a/b = one combination pair. Sort by smallest distances between points on contour
  std::sort(id_comb_r.begin(), id_comb_r.end(), 
      [&features_r_t](const std::vector<uint8_t> &a, const std::vector<uint8_t> &b) -> bool 
      {
        auto diffx = features_r_t[a[0]].x - features_r_t[a[1]].x;
        auto diffy = features_r_t[a[0]].y - features_r_t[a[1]].y;
        auto distanceA = std::sqrt(std::pow(diffx, 2) + std::pow(diffy, 2));

        diffx = features_r_t[b[0]].x - features_r_t[b[1]].x;
        diffy = features_r_t[b[0]].y - features_r_t[b[1]].y;
        auto distanceB = std::sqrt(std::pow(diffx, 2) + std::pow(diffy, 2));

        return distanceA < distanceB;
      });

  
  for(int i = 0; i < 4; ++i)
  {
    features_r.push_back((features_r_t[id_comb_r[i][0]] + features_r_t[id_comb_r[i][1]])/2);
  }

  // std::vector<cv::Point> features_r_t_t;

  // for(int i = 0; i < 2; ++i)
  // {
  //   features_r_t_t.push_back(features_r_t[id_comb_l[i][0]]);
  //   features_r_t_t.push_back(features_r_t[id_comb_l[i][1]]);
  // }

  // features_r.push_back(cv::Point((features_r_t_t[0].x + features_r_t_t[0].x)/2, features_r_t_t[0].y));
  // features_r.push_back(cv::Point((features_r_t_t[1].x + features_r_t_t[1].x)/2, features_r_t_t[1].y));
  // features_r.push_back(cv::Point(features_r_t_t[0].x, (features_r_t_t[0].y + features_r_t_t[0].y)/2));
  // features_r.push_back(cv::Point(features_r_t_t[1].x, (features_r_t_t[1].y + features_r_t_t[1].y)/2));




  // Calculate stereo correspondence
  // return;
  std::vector<int> correspondence_pair_idxs;
  // stereo_correspondence(l_diffused, r_diffused, features_l, features_r, correspondence_pair_idxs);
  // Dumb stereo matching
  std::cout << "Stereo matching..." << std::endl;
  {
    double curr_min_dist, xdiff, ydiff, dist;
    int curr_min_dist_idx;
    int y_diff_thresh = current_image_left.rows * 0.02;
    std::cout << "y_diff_thresh: " << y_diff_thresh << std::endl;
    for (size_t i = 0; i < features_l.size(); i++)
    {
      curr_min_dist_idx = -1;
      curr_min_dist = 1E6;
      // std::cout << "\x1b[31m" << i << " \x1b[0mCurrent pt: " << features_l[i] << std::endl;
      for (size_t j = 0; j < features_r.size(); j++)
      {
        // std::cout << "\t\x1b[31m" << j << " \x1b[0mCandidate pt: " << features_r[j] << std::endl;
        ydiff = features_l[i].y - features_r[j].y;
        // std::cout << "\t   ydiff: " << ydiff << std::endl;
        if (abs(ydiff) > y_diff_thresh)
          continue;
        xdiff = features_l[i].x - features_r[j].x;

        dist = sqrt(xdiff * xdiff + ydiff * ydiff);
        std::cout << "\t   dist: " << dist << " left = " << i << " right = " << j << std::endl;
        if (dist < curr_min_dist)
        {
          curr_min_dist = dist;
          curr_min_dist_idx = j;
        }
      }
      correspondence_pair_idxs.push_back(curr_min_dist_idx);
      std::cout << "Match: " << curr_min_dist_idx << std::endl;
    }
  }
  std::vector<cv::Scalar> colors = {cv::Scalar(255, 0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255), cv::Scalar(255,255,255)};

  for (int i = 0; i < features_l.size(); ++i)
  {
    cv::circle(current_image_left, features_l[i], 10, colors[i], -1);
    cv::circle(current_image_right, features_r[correspondence_pair_idxs[i]], 10, colors[i], -1);

  }



  if(std::count(correspondence_pair_idxs.begin(), correspondence_pair_idxs.end(), -1) != 0) return;



  // Print correspondences
  for (size_t i = 0; i < correspondence_pair_idxs.size(); i++)
  {
    std::cout << i << " <--> " << correspondence_pair_idxs[i] << std::endl;
  }

  // Get camera projection matrices
  cv::Matx34d left_cam_mat = left_cam_model.fullProjectionMatrix();
  cv::Matx34d right_cam_mat = right_cam_model.fullProjectionMatrix();

  // Calculate 3D stereo reconstructions
  std::vector<Eigen::Vector3d> feature_pts_3d;
  Eigen::Vector3d pt_3D;
  // double reset_scaling = 1 / image_proc_scale;
  std::cout << "feature reconstructions(3D):\n";
  for (size_t i = 0; i < correspondence_pair_idxs.size(); i++)
  {
    if (correspondence_pair_idxs[i] == -1)
      continue;
    cv::Point2d pt_L = features_l[i];
    cv::Point2d pt_R = features_r[correspondence_pair_idxs[i]];

    // Undo the effects of working with coordinates from scaled images
    // pt_L = pt_L * reset_scaling;
    // pt_R = pt_R * reset_scaling;

    // Print points in image coordinates
    std::cout << "L: " << pt_L << "R: " << pt_R << std::endl;

    cv::Matx31d pt_L_hom(pt_L.x, pt_L.y, 1);
    cv::Matx31d pt_R_hom(pt_R.x, pt_R.y, 1);
    cv::Mat X_hom = mil_vision::triangulate_Linear_LS(cv::Mat(left_cam_mat), cv::Mat(right_cam_mat), cv::Mat(pt_L_hom), cv::Mat(pt_R_hom));
    X_hom = X_hom / X_hom.at<double>(3, 0);
    pt_3D << X_hom.at<double>(0, 0), X_hom.at<double>(1, 0), X_hom.at<double>(2, 0);
    if(pt_3D(2) < 0) return;
    if(pt_3D(2) > 5) return;
    std::cout << "[ " << pt_3D(0) << ", " << pt_3D(1) << ", " << pt_3D(2) << "]" << std::endl;
    feature_pts_3d.push_back(pt_3D);
  }
  std::cout << "num 3D features: " << feature_pts_3d.size() << std::endl;


  visualization_msgs::Marker points_raw;
  points_raw.header.stamp = ros::Time::now();
  points_raw.type = visualization_msgs::Marker::POINTS;
  points_raw.header.frame_id = "/front_left_cam";
  points_raw.id = 5;
  points_raw.scale.x = 0.2;
  points_raw.scale.y = 0.2;
  points_raw.color.r = 1.0f;
  points_raw.color.a = 1.0;

  // visualize reconstructions
  for (size_t i = 0; i < feature_pts_3d.size(); i++)
  {

    Eigen::Vector3d pt = feature_pts_3d[i];
    cv::Matx41d position_hom(pt(0), pt(1), pt(2), 1);
    cv::Matx31d pt_L_2d_hom = left_cam_mat * position_hom;
    cv::Point2d L_center2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));
    cv::Scalar color(255, 0, 255);
    std::stringstream label;
    label << i;
    std::cout << "L_center2d: " << L_center2d << std::endl;
    std::cout << pt_L_2d_hom << std::endl;
    cv::circle(current_image_left, L_center2d, 5, color, -1);
    cv::putText(current_image_left, label.str(), L_center2d, CV_FONT_HERSHEY_SIMPLEX, 0.0015 * current_image_left.rows,
            cv::Scalar(0, 0, 0), 2);

    cv::Matx31d pt_R_2d_hom = right_cam_mat * position_hom;
    cv::Point2d R_center2d(pt_R_2d_hom(0) / pt_R_2d_hom(2), pt_R_2d_hom(1) / pt_R_2d_hom(2));
    cv::circle(current_image_right, R_center2d, 5, color, -1);
    cv::putText(current_image_right, label.str(), R_center2d, CV_FONT_HERSHEY_SIMPLEX, 0.0015 * current_image_right.rows,
            cv::Scalar(0, 0, 0), 2);
     geometry_msgs::Point p;
      p.x = feature_pts_3d[i][0];
      p.y = feature_pts_3d[i][1];
      p.z = feature_pts_3d[i][2];
      points_raw.points.push_back(p);
  }

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

  std::cout << "determinant: " << abs(matrix_of_vectors.determinant()*100) << std::endl;
  if (abs(matrix_of_vectors.determinant())*100 > 30) return;

   // Calculate best fit plane
  Eigen::Matrix<double, 4, 3> A;
  Eigen::Matrix<double, 4, 1> b_vec;
  A << feature_pts_3d[0][0], feature_pts_3d[0][1], feature_pts_3d[0][2],
       feature_pts_3d[1][0], feature_pts_3d[1][1], feature_pts_3d[1][2],
       feature_pts_3d[2][0], feature_pts_3d[2][1], feature_pts_3d[2][2],
       feature_pts_3d[3][0], feature_pts_3d[3][1], feature_pts_3d[3][2];
  b_vec << 1, 1, 1, 1;
  Eigen::Matrix<double, 3, 1> x = A.colPivHouseholderQr().solve(b_vec);
  double a, b, c, d;
  a = 1;
  b = x[1] / x[0];
  c = x[2] / x[0];
  d = -1 / x[0];
  std::cout << "best fit plane: z = " << -a / c << "x + " << -b / c << "y + " << -d / c << std::endl;

  visualization_msgs::Marker points;
  points.header.stamp = ros::Time::now();
  points.type = visualization_msgs::Marker::POINTS;
  points.header.frame_id = "/front_left_cam";
  points.id = 0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Project points to best fit plane
  std::vector<Eigen::Vector3d> proj_pts;
  Eigen::Vector3d plane_unit_normal;
  plane_unit_normal << a, b, c;
  plane_unit_normal = plane_unit_normal / plane_unit_normal.norm();
  Eigen::Vector3d pt_on_plane;
  pt_on_plane << 0, 0, -d / c;
  for (uint8_t pt_idx = 0; pt_idx < feature_pts_3d.size(); ++pt_idx)
  {
    Eigen::Vector3d pt = feature_pts_3d[pt_idx];
    Eigen::Vector3d plane_to_pt_vec = pt - pt_on_plane;
    Eigen::Vector3d plane_to_pt_proj_normal = plane_to_pt_vec.dot(plane_unit_normal) * plane_unit_normal;
    Eigen::Vector3d corr_pt = pt - plane_to_pt_proj_normal;
    std::cout << (int)pt_idx << ":\noriginal pt: [" << pt[0] << ", " << pt[1] << ", " << pt[2] << "] \ncorrected: ["
         << corr_pt[0] << ", " << corr_pt[1] << ", " << corr_pt[2] << "]\n\tdist: " << (pt - corr_pt).norm() << std::endl;
    proj_pts.push_back(corr_pt);
          geometry_msgs::Point p;
      p.x = corr_pt[0];
      p.y = corr_pt[1];
      p.z = corr_pt[2];
      points.points.push_back(p);
  }

  geometry_msgs::Point center_pt;
  for(auto p : proj_pts)
  {
    center_pt.x += p[0];
    center_pt.y += p[1];
    center_pt.z += p[2];
  }
  center_pt.x/=proj_pts.size();
  center_pt.y/=proj_pts.size();
  center_pt.z/=proj_pts.size();

  points.points.push_back(center_pt);

  geometry_msgs::Point sdp_normalvec_ros;
  sdp_normalvec_ros.x = center_pt.x + plane_unit_normal(0,0);
  sdp_normalvec_ros.y = center_pt.y + plane_unit_normal(1,0);
  sdp_normalvec_ros.z = center_pt.z + plane_unit_normal(2,0);

  geometry_msgs::Vector3 normal_vec;
  normal_vec.x = plane_unit_normal(0,0);
  normal_vec.y = plane_unit_normal(1,0);
  normal_vec.z = plane_unit_normal(2,0);


  center_gate_pub.publish(center_pt);
  normal_gate_pub.publish(normal_vec);


  visualization_msgs::Marker marker_normal;
  marker_normal.header.seq = 0;
  marker_normal.header.stamp = ros::Time::now();
  marker_normal.header.frame_id = "/front_left_cam";
  marker_normal.id = 3000;
  marker_normal.type = visualization_msgs::Marker::ARROW;
  marker_normal.points.push_back(center_pt);
  marker_normal.points.push_back(sdp_normalvec_ros);
  marker_normal.scale.x = 0.1;
  marker_normal.scale.y = 0.5;
  marker_normal.scale.z = 0.5;
  marker_normal.color.a = 1.0;
  marker_normal.color.r = 0.0;
  marker_normal.color.g = 0.0;
  marker_normal.color.b = 1.0;
//fuck
  // double dist_to_centroid_relative_error_thresh = 0.2;
  // double model_height = 1.9304;
  // double model_width = 1.0033;
  // double dist_to_centroid_ideal = sqrt(model_height * model_height / 4 + model_width * model_width / 4);
  // double sum_network_distances_ideal =
  //     2 * sqrt(model_height * model_height + model_width * model_width) + 2 * model_height + 2 * model_width;

  // std::cout << "Dis to centroid: " << dist_to_centroid_ideal << std::endl;
  // std::cout << "sum_network_distances_ideal: " << sum_network_distances_ideal << std::endl;

  // double xsum = 0;
  // double ysum = 0;
  // double zsum = 0;
  // double dist_to_centroid_avg = 0;
  // double xdiff, ydiff, zdiff;

  // for(auto a : feature_pts_3d)
  // {
  //   xsum += a(0);
  //   ysum += a(1);
  //   zsum += a(2);
  // }

  // for(auto a : feature_pts_3d)
  // {
  //   xdiff = a(0) - xsum / 4.0;
  //   ydiff = a(1) - ysum / 4.0;
  //   zdiff = a(2) - zsum / 4.0;
  //   dist_to_centroid_avg += sqrt(xdiff * xdiff + ydiff * ydiff + zdiff * zdiff);
  // }
  // dist_to_centroid_avg /= 4.0;
  // double dist_to_centroid_avg_error = fabs(dist_to_centroid_avg - dist_to_centroid_ideal) / dist_to_centroid_ideal;
  // std::cout << "\tdist_to_centroid_avg: " << dist_to_centroid_avg << std::endl;
  // std::cout << "\tdist_error: " << dist_to_centroid_avg_error << std::endl;
  // if(dist_to_centroid_avg_error > dist_to_centroid_relative_error_thresh) return;


    if( !marker_pub ) {
      ROS_WARN("Publisher invalid!");
    }
    else {
      marker_pub.publish(points);
      marker_pub.publish(points_raw);
      marker_pub.publish(marker_normal);
    }
  // cv::imshow("Detection-left", current_image_left);
  // cv::imshow("Detection-right", current_image_right);

  // cv::waitKey(30);

  sensor_msgs::ImagePtr dbg_img_msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image_left).toImageMsg();
  debug_image_pub_left.publish(dbg_img_msg_left);

  sensor_msgs::ImagePtr dbg_img_msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image_right).toImageMsg();
  debug_image_pub_right.publish(dbg_img_msg_right);


}

double Sub8StartGateDetector::get_angle(cv::Point a, cv::Point b, cv::Point c)
{
  auto ba = a - b;
  auto bc = c - b;

  auto cos_ang = ba.dot(bc) / cv::norm(ba) * cv::norm(bc);
  auto angle = acos(cos_ang);

  return angle * 180 / CV_PI;
}

bool Sub8StartGateDetector::valid_contour(std::vector<cv::Point> &contour)
{
  if(cv::isContourConvex(contour)) return false;
  auto area = cv::contourArea(contour);
  if (area < 3000) return false;
  if (area > 30000) return false;
  auto epsilon = 0.01 * cv::arcLength(contour, true);
  std::vector<cv::Point> approx;
  cv::approxPolyDP(contour, approx, epsilon, true);
  cv::Moments mu = cv::moments(contour, false);
  auto center = cv::Point(mu.m10/mu.m00 , mu.m01/mu.m00);

  if(cv::pointPolygonTest(contour, center, false) == 1) return false;
  // auto rect = cv::minAreaRect(approx);
  // if(rect[1][0] * rect[1][1] < 7000) return false;

  if(approx.size() < 8) return false;
  if(approx.size() > 13) return false;

  for (size_t i = 0; i < approx.size(); i+=2)
  {
    auto angle = get_angle(approx[i], approx[i+1], approx[i+2]);
    if (abs(angle - 90) > 23) return false;
  }
  auto angle = get_angle(approx[approx.size()-1], approx[0], approx[1]);
  if (abs(angle - 90) > 23) return false;

  return true;
}


void Sub8StartGateDetector::combinations(uint8_t n, uint8_t k, std::vector<std::vector<uint8_t> > &idx_array)
{
  idx_array = std::vector<std::vector<uint8_t> >();

  std::vector<uint8_t> first_comb;

  // set first combination indices
  for (uint8_t i = 0; i < k; i++)
  {
    first_comb.push_back(i);
  }

  uint8_t level = 0;

  _increase_elements_after_level(first_comb, idx_array, n, k, level);
}

void Sub8StartGateDetector::_increase_elements_after_level(std::vector<uint8_t> comb, std::vector<std::vector<uint8_t> > &comb_array, uint8_t n, uint8_t k,
                                    uint8_t level)
{
  std::vector<uint8_t> parent = comb;
  std::vector<std::vector<uint8_t> > children;

  while (true)
  {
    for (uint8_t idx = level; idx < k; idx++)
    {
      comb[idx] = comb[idx] + 1;
    }
    if (comb[level] > n - (k - level))
      break;
    children.push_back(comb);
  }

  if (level == k - 1)
  {
    comb_array.push_back(parent);
    for (std::vector<uint8_t> child : children)
    {
      comb_array.push_back(child);
    }
  }
  else
  {
    _increase_elements_after_level(parent, comb_array, n, k, level + 1);
    for (std::vector<uint8_t> child : children)
    {
      _increase_elements_after_level(child, comb_array, n, k, level + 1);
    }
  }
}

