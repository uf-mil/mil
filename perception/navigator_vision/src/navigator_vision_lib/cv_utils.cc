#include <navigator_vision_lib/cv_tools.hpp>

namespace nav {

cv::Point contour_centroid(Contour &contour) {
  cv::Moments m = cv::moments(contour, true);
  cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
  return center;
}

bool larger_contour(const Contour &c1, const Contour &c2) {
  if (cv::contourArea(c1) > cv::contourArea(c2))
    return true;
  else
    return false;
}

cv::MatND smooth_histogram(const cv::MatND &histogram,
                           size_t filter_kernel_size, float sigma) {
  cv::MatND hist = histogram.clone();
  std::vector<float> gauss_kernel =
      generate_gaussian_kernel_1D(filter_kernel_size, sigma);
  size_t histSize = hist.total();
  int offset = (filter_kernel_size - 1) / 2;
  for (size_t i = offset; i < histSize - offset;
       i++) // Convolve histogram values with gaussian kernel
  {
    int sum = 0;
    int kernel_idx = 0;
    for (int j = i - offset; j <= int(i + offset); j++) {
      sum += (hist.at<float>(j) * gauss_kernel[kernel_idx++]);
    }
    hist.at<float>(i) = sum;
  }
  for (int i = 0; i < offset; ++i) // Pad filtered result with zeroes
  {
    hist.at<float>(i) = 0;
    hist.at<float>(histSize - 1 - i) = 0;
  }
  return hist;
}

std::vector<float> generate_gaussian_kernel_1D(size_t kernel_size,
                                               float sigma) {
  std::vector<float> kernel;
  int middle_index = (kernel_size - 1) / 2;
  int first_discrete_sample_x = -(middle_index);
  for (int i = first_discrete_sample_x; i <= 0; i++) {
    float power = -0.5 * (float(i) / sigma) * (float(i) / sigma);
    kernel.push_back(
        exp(power)); // From definition of Standard Normal Distribution
  }
  for (int i = 1; i <= middle_index; i++) { // Kernel is symmetric
    kernel.push_back(kernel[middle_index - i]);
  }
  // Normalize kernel (sum of values should equal 1.0)
  float sum = 0;
  for (size_t i = 0; i < kernel_size; i++) {
    sum += kernel[i];
  }
  for (size_t i = 0; i < kernel_size; i++) {
    kernel[i] /= sum;
  }
  return kernel;
}

std::vector<cv::Point> find_local_maxima(const cv::MatND &histogram,
                                         float thresh_multiplier) {

  std::stringstream ros_log;
  ros_log << "\x1b[1;31m"
          << "find_local_maxima"
          << "\x1b[0m" << std::endl;

  std::vector<cv::Point> local_maxima, threshed_local_maxima;
  float global_maximum = -std::numeric_limits<double>::infinity();

  // Locate local maxima and find global maximum
  for (size_t idx = 1; idx < histogram.total() - 1; idx++) {
    float current_value = histogram.at<float>(idx);
    if ((histogram.at<float>(idx - 1) < current_value) &&
        (histogram.at<float>(idx + 1) <= current_value)) {
      local_maxima.push_back(cv::Point(idx, current_value));
      if (global_maximum < current_value)
        global_maximum = current_value;
    }
  }
  ros_log << "Maxima [x, y]:";
  for (size_t i = 0; i < local_maxima.size(); i++) {
    if (local_maxima[i].y > global_maximum * thresh_multiplier)
      threshed_local_maxima.push_back(local_maxima[i]);
    if (i % 4 == 0)
      ros_log << std::endl
              << "\t";
    ros_log << "[" << std::setw(5) << local_maxima[i].x << "," << std::setw(5)
            << local_maxima[i].y << "] ";
  }
  ros_log << std::endl
          << "thresh: > global_maximum(" << global_maximum
          << ") * thresh_multiplier(" << thresh_multiplier
          << ") = " << (global_maximum * thresh_multiplier);
  ros_log << std::endl
          << "Threshed Maxima (x): ";
  if (threshed_local_maxima.size() != local_maxima.size()) {
    BOOST_FOREACH (cv::Point pt, threshed_local_maxima) {
      ros_log << " " << pt.x << " ";
    }
  } else
    ros_log << "same as 'Maxima'";
  ros_log << std::endl;
#ifdef SEGMENTATION_DEBUG
  ROS_INFO(ros_log.str().c_str());
#endif
  return threshed_local_maxima;
}

std::vector<cv::Point> find_local_minima(const cv::MatND &histogram,
                                         float thresh_multiplier) {

  std::stringstream ros_log;
  ros_log << "\x1b[1;31m"
          << "find_local_minima"
          << "\x1b[0m" << std::endl;

  std::vector<cv::Point> local_minima, threshed_local_minima;
  float global_minimum = std::numeric_limits<double>::infinity();
  ;

  // Locate local minima and find global minimum
  for (size_t idx = 1; idx < histogram.total() - 1; idx++) {
    float current_value = histogram.at<float>(idx);
    if ((histogram.at<float>(idx - 1) >= current_value) &&
        (histogram.at<float>(idx + 1) > current_value)) {
      local_minima.push_back(cv::Point(idx, current_value));
      if (global_minimum > current_value)
        global_minimum = current_value;
    }
  }
  ros_log << "Minima [x, y]:";
  for (size_t i = 0; i < local_minima.size(); i++) {
    if (local_minima[i].y < global_minimum * thresh_multiplier)
      threshed_local_minima.push_back(local_minima[i]);
    if (i % 4 == 0)
      ros_log << std::endl
              << "\t";
    ros_log << "[" << std::setw(5) << local_minima[i].x << "," << std::setw(5)
            << local_minima[i].y << "] ";
  }
  ros_log << std::endl
          << "thresh: < global_minimum(" << global_minimum
          << ") * thresh_multiplier(" << thresh_multiplier
          << ") = " << (global_minimum * thresh_multiplier);
  ros_log << std::endl
          << "Threshed Minima (x): ";
  if (threshed_local_minima.size() != local_minima.size()) {
    BOOST_FOREACH (cv::Point pt, threshed_local_minima) {
      ros_log << " " << pt.x << " ";
    }
  } else
    ros_log << "same as 'Minima'";
  ros_log << std::endl;
#ifdef SEGMENTATION_DEBUG
  ROS_INFO(ros_log.str().c_str());
#endif
  return threshed_local_minima;
}

unsigned int select_hist_mode(std::vector<cv::Point> &histogram_modes,
                              int target) {

  std::stringstream ros_log;
  ros_log << "\x1b[1;31m"
          << "select_hist_mode"
          << "\x1b[0m" << std::endl;

  std::vector<int> distances;
  BOOST_FOREACH (cv::Point mode, histogram_modes) {
    distances.push_back(mode.x - target);
  }
  int min_idx = 0;
  for (size_t i = 0; i < distances.size(); i++) {
    if (std::abs(distances[i]) <= std::abs(distances[min_idx]))
      min_idx = i;
  }
  if (histogram_modes.size() == 0) {
    ros_log << "No modes could be generated" << std::endl;
    ROS_INFO(ros_log.str().c_str());
    return 0;
  } else
    return histogram_modes[min_idx].x;
}

void statistical_image_segmentation(const cv::Mat &src, cv::Mat &dest,
                                    cv::Mat &debug_img, const int hist_size,
                                    const float **ranges, const int target,
                                    std::string image_name, bool ret_dbg_img,
                                    const float sigma,
                                    const float low_thresh_gain,
                                    const float high_thresh_gain) {
  std::stringstream ros_log;
  ros_log << "\x1b[1;31m"
          << "statistical_image_segmentation"
          << "\x1b[0m" << std::endl;

  // Calculate histogram
  cv::MatND hist, hist_smooth, hist_derivative;
  cv::calcHist(&src, 1, 0, cv::Mat(), hist, 1, &hist_size, ranges, true, false);

  // Smooth histogram
  const int kernel_size = 11;
  hist_smooth = nav::smooth_histogram(hist, kernel_size, sigma);

  // Calculate histogram derivative (central finite difference)
  hist_derivative = hist_smooth.clone();
  hist_derivative.at<float>(0) = 0;
  hist_derivative.at<float>(hist_size - 1) = 0;
  for (int i = 1; i < hist_size - 1; ++i) {
    hist_derivative.at<float>(i) =
        (hist_smooth.at<float>(i + 1) - hist_smooth.at<float>(i - 1)) / 2.0;
  }
  hist_derivative = nav::smooth_histogram(hist_derivative, kernel_size, sigma);

  // Find target mode
  std::vector<cv::Point> histogram_modes =
      nav::find_local_maxima(hist_smooth, 0.1);
  int target_mode = nav::select_hist_mode(histogram_modes, target);
  ros_log << "Target: " << target << std::endl;
  ros_log << "Mode Selected: " << target_mode << std::endl;

  // Calculate std dev of histogram slopes
  cv::Scalar hist_deriv_mean, hist_deriv_stddev;
  cv::meanStdDev(hist_derivative, hist_deriv_mean, hist_deriv_stddev);

  // Determine thresholds for cv::inRange() using the std dev of histogram
  // slopes times a gain as a cutoff heuristic
  int high_abs_derivative_thresh =
      std::abs(hist_deriv_stddev[0] * high_thresh_gain);
  int low_abs_derivative_thresh =
      std::abs(hist_deriv_stddev[0] * low_thresh_gain);
  std::vector<cv::Point> derivative_maxima =
      nav::find_local_maxima(hist_derivative, 0.01);
  std::vector<cv::Point> derivative_minima =
      nav::find_local_minima(hist_derivative, 0.01);
  int high_thresh_search_start = target_mode;
  int low_thresh_search_start = target_mode;
  ros_log << "high_thresh_search_start: " << target_mode << std::endl;
  ros_log << "Looking for the local minimum of the derivative of the histogram "
             "immediately to the right of the selected mode." << std::endl;

  for (size_t i = 0; i < derivative_minima.size(); i++) {
    ros_log << "\tderivative_minima[" << i << "].x = " << derivative_minima[i].x
            << std::endl;
    if (derivative_minima[i].x > target_mode) {
      high_thresh_search_start = derivative_minima[i].x;
      ros_log << "Done: The upper threshold will be no less than "
              << high_thresh_search_start << std::endl;
      break;
    }
  }
  ros_log << "low_thresh_search_start: " << target_mode << std::endl;
  ros_log << "Looking for the local maximum of the derivative of the histogram "
             "immediately to the left of the selected mode." << std::endl;
  for (int i = derivative_maxima.size() - 1; i >= 0; i--) {
    ros_log << "\tderivative_maxima[" << i << "].x = " << derivative_maxima[i].x
            << std::endl;
    if (derivative_maxima[i].x < target_mode) {
      low_thresh_search_start = derivative_maxima[i].x;
      ros_log << "Done: The lower threshold will be no greater than "
              << low_thresh_search_start << std::endl;
      break;
    }
  }
  int high_thresh = high_thresh_search_start;
  int low_thresh = low_thresh_search_start;
  ros_log << "high_deriv_thresh: " << hist_deriv_stddev[0] << " * "
          << high_thresh_gain << " = " << high_abs_derivative_thresh
          << std::endl;
  ros_log << "abs(high_deriv_thresh) - abs(slope) = slope_error" << std::endl;
  ros_log << "i: potential upper threshold" << std::endl;
  ros_log << "Looking for first i such that slope_error >= 0" << std::endl;
  for (int i = high_thresh_search_start; i < hist_size; i++) {
    int abs_slope = std::abs(hist_derivative.at<float>(i));
    ros_log << "\ti = " << i << "  :  " << high_abs_derivative_thresh << " - "
            << abs_slope << " = " << high_abs_derivative_thresh - abs_slope
            << std::endl;
    if (abs_slope <= high_abs_derivative_thresh) {
      high_thresh = i;
      break;
    }
  }
  ros_log << "high_thresh = " << high_thresh << std::endl;
  ros_log << "low_deriv_thresh: " << hist_deriv_stddev[0] << " * "
          << low_thresh_gain << " = " << low_abs_derivative_thresh << std::endl;
  ros_log << "abs(low_deriv_thresh) - abs(slope) = slope_error" << std::endl;
  ros_log << "i: potential lower threshold" << std::endl;
  ros_log << "Looking for first i such that slope_error <= 0" << std::endl;
  for (int i = low_thresh_search_start; i > 0; i--) {
    int abs_slope = std::abs(hist_derivative.at<float>(i));
    ros_log << "\ti = " << i << "  :  " << low_abs_derivative_thresh << " - "
            << abs_slope << " = " << high_abs_derivative_thresh - abs_slope
            << std::endl;

    if (abs_slope <= low_abs_derivative_thresh) {
      low_thresh = i;
      break;
    }
  }

  ros_log << "low_thresh = " << low_thresh << std::endl;
  ros_log << "\x1b[1;37mTarget: " << target
          << "\nClosest distribution mode: " << target_mode
          << "\nThresholds selected:  low=" << low_thresh
          << "  high=" << high_thresh << "\x1b[0m" << std::endl;

  // Threshold image
  cv::inRange(src, low_thresh, high_thresh, dest);

#ifdef SEGMENTATION_DEBUG
  ROS_INFO(ros_log.str().c_str());
  cv::imshow("src" + image_name, src);
  cv::waitKey(1);
#endif

  // Closing Morphology operation
  int dilation_size = 2;
  cv::Mat structuring_element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
      cv::Point(dilation_size, dilation_size));
  cv::dilate(dest, dest, structuring_element);
  cv::erode(dest, dest, structuring_element);

  if (ret_dbg_img) {

    try {
      // Prepare to draw graph of histogram and derivative
      int hist_w = src.cols;
      int hist_h = src.rows;
      int bin_w = hist_w / hist_size;
      cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(0, 0, 0));
      cv::Mat histDerivImage(hist_h, hist_w, CV_8UC1, cv::Scalar(0, 0, 0));
      cv::normalize(hist_smooth, hist_smooth, 0, histImage.rows,
                    cv::NORM_MINMAX, -1, cv::Mat());
      cv::normalize(hist_derivative, hist_derivative, 0, histImage.rows,
                    cv::NORM_MINMAX, -1, cv::Mat());

      // Draw Graphs
      for (int i = 1; i < hist_size; i++) {
        // Plot image histogram
        cv::line(
            histImage,
            cv::Point(bin_w * (i - 1),
                      hist_h - cvRound(hist_smooth.at<float>(i - 1))),
            cv::Point(bin_w * (i), hist_h - cvRound(hist_smooth.at<float>(i))),
            cv::Scalar(255, 0, 0), 2, 8, 0);
        // Plot image histogram derivative
        cv::line(histDerivImage,
                 cv::Point(bin_w * (i - 1),
                           hist_h - cvRound(hist_derivative.at<float>(i - 1))),
                 cv::Point(bin_w * (i),
                           hist_h - cvRound(hist_derivative.at<float>(i))),
                 cv::Scalar(122, 0, 0), 1, 8, 0);
      }

      // Shade in area being segmented under histogram curve
      cv::line(histImage,
               cv::Point(bin_w * low_thresh,
                         hist_h - cvRound(hist_smooth.at<float>(low_thresh))),
               cv::Point(bin_w * low_thresh, hist_h), cv::Scalar(255, 255, 0),
               2);
      cv::line(histImage,
               cv::Point(bin_w * high_thresh,
                         hist_h - cvRound(hist_smooth.at<float>(high_thresh))),
               cv::Point(bin_w * high_thresh, hist_h), cv::Scalar(255, 255, 0),
               2);
      cv::floodFill(
          histImage,
          cv::Point(bin_w * cvRound(float(low_thresh + high_thresh) / 2.0),
                    hist_h - 1),
          cv::Scalar(155));

      // Combine graphs into one image and display results
      cv::Mat segmentation_channel = cv::Mat::zeros(histImage.size(), CV_8UC1);
      std::vector<cv::Mat> debug_img_channels;
      debug_img_channels.push_back(histImage);
      debug_img_channels.push_back(histDerivImage);
      debug_img_channels.push_back(dest * 0.25);
      cv::merge(debug_img_channels, debug_img);
      cv::Point text_upper_left(debug_img.cols / 2.0, debug_img.rows / 10.0);
      std::string text_ln1 = image_name;
      std::stringstream text_ln2;
      text_ln2 << "low = " << low_thresh;
      std::stringstream text_ln3;
      text_ln3 << "high = " << high_thresh;
      int font = cv::FONT_HERSHEY_SIMPLEX;
      double font_scale = 0.0015 * debug_img.rows;
      cv::Point vert_offset = cv::Point(0, debug_img.rows / 15.0);
      cv::Scalar text_color(255, 255, 0);
      cv::putText(debug_img, text_ln1, text_upper_left, font, font_scale,
                  text_color);
      cv::putText(debug_img, text_ln2.str(), text_upper_left + vert_offset,
                  font, font_scale, text_color);
      cv::putText(debug_img, text_ln3.str(),
                  text_upper_left + vert_offset + vert_offset, font, font_scale,
                  text_color);
    } catch (std::exception &e) {
      ROS_INFO(e.what());
    }
  }
}

cv::Mat triangulate_Linear_LS(cv::Mat mat_P_l, cv::Mat mat_P_r,
                              cv::Mat undistorted_l, cv::Mat undistorted_r) {
  cv::Mat A(4, 3, CV_64FC1), b(4, 1, CV_64FC1), X(3, 1, CV_64FC1),
      X_homogeneous(4, 1, CV_64FC1), W(1, 1, CV_64FC1);
  W.at<double>(0, 0) = 1.0;
  A.at<double>(0, 0) =
      (undistorted_l.at<double>(0, 0) / undistorted_l.at<double>(2, 0)) *
          mat_P_l.at<double>(2, 0) -
      mat_P_l.at<double>(0, 0);
  A.at<double>(0, 1) =
      (undistorted_l.at<double>(0, 0) / undistorted_l.at<double>(2, 0)) *
          mat_P_l.at<double>(2, 1) -
      mat_P_l.at<double>(0, 1);
  A.at<double>(0, 2) =
      (undistorted_l.at<double>(0, 0) / undistorted_l.at<double>(2, 0)) *
          mat_P_l.at<double>(2, 2) -
      mat_P_l.at<double>(0, 2);
  A.at<double>(1, 0) =
      (undistorted_l.at<double>(1, 0) / undistorted_l.at<double>(2, 0)) *
          mat_P_l.at<double>(2, 0) -
      mat_P_l.at<double>(1, 0);
  A.at<double>(1, 1) =
      (undistorted_l.at<double>(1, 0) / undistorted_l.at<double>(2, 0)) *
          mat_P_l.at<double>(2, 1) -
      mat_P_l.at<double>(1, 1);
  A.at<double>(1, 2) =
      (undistorted_l.at<double>(1, 0) / undistorted_l.at<double>(2, 0)) *
          mat_P_l.at<double>(2, 2) -
      mat_P_l.at<double>(1, 2);
  A.at<double>(2, 0) =
      (undistorted_r.at<double>(0, 0) / undistorted_r.at<double>(2, 0)) *
          mat_P_r.at<double>(2, 0) -
      mat_P_r.at<double>(0, 0);
  A.at<double>(2, 1) =
      (undistorted_r.at<double>(0, 0) / undistorted_r.at<double>(2, 0)) *
          mat_P_r.at<double>(2, 1) -
      mat_P_r.at<double>(0, 1);
  A.at<double>(2, 2) =
      (undistorted_r.at<double>(0, 0) / undistorted_r.at<double>(2, 0)) *
          mat_P_r.at<double>(2, 2) -
      mat_P_r.at<double>(0, 2);
  A.at<double>(3, 0) =
      (undistorted_r.at<double>(1, 0) / undistorted_r.at<double>(2, 0)) *
          mat_P_r.at<double>(2, 0) -
      mat_P_r.at<double>(1, 0);
  A.at<double>(3, 1) =
      (undistorted_r.at<double>(1, 0) / undistorted_r.at<double>(2, 0)) *
          mat_P_r.at<double>(2, 1) -
      mat_P_r.at<double>(1, 1);
  A.at<double>(3, 2) =
      (undistorted_r.at<double>(1, 0) / undistorted_r.at<double>(2, 0)) *
          mat_P_r.at<double>(2, 2) -
      mat_P_r.at<double>(1, 2);
  b.at<double>(0, 0) =
      -((undistorted_l.at<double>(0, 0) / undistorted_l.at<double>(2, 0)) *
            mat_P_l.at<double>(2, 3) -
        mat_P_l.at<double>(0, 3));
  b.at<double>(1, 0) =
      -((undistorted_l.at<double>(1, 0) / undistorted_l.at<double>(2, 0)) *
            mat_P_l.at<double>(2, 3) -
        mat_P_l.at<double>(1, 3));
  b.at<double>(2, 0) =
      -((undistorted_r.at<double>(0, 0) / undistorted_r.at<double>(2, 0)) *
            mat_P_r.at<double>(2, 3) -
        mat_P_r.at<double>(0, 3));
  b.at<double>(3, 0) =
      -((undistorted_r.at<double>(1, 0) / undistorted_r.at<double>(2, 0)) *
            mat_P_r.at<double>(2, 3) -
        mat_P_r.at<double>(1, 3));
  solve(A, b, X, cv::DECOMP_SVD);
  vconcat(X, W, X_homogeneous);
  return X_homogeneous;
}

Eigen::Vector3d kanatani_triangulation(const cv::Point2d &pt1,
                                       const cv::Point2d &pt2,
                                       const Eigen::Matrix3d &essential,
                                       const Eigen::Matrix3d &R) {
  /*
          K. Kanatani, Y. Sugaya, and H. Niitsuma. Triangulation from two views
     revisited: Hartley-Sturm vs. optimal
          correction. In British Machine Vision Conference, page 55, 2008.
  */
  // std::cout << "ptL_noisy: " << pt1 << " ptR_noisy: " << pt2 << std::endl;
  const unsigned int max_iterations = 7;
  Eigen::Vector3d p1_old(pt1.x, pt1.y, 1.0);
  Eigen::Vector3d p2_old(pt2.x, pt2.y, 1.0);
  const Eigen::Vector3d p1_0(pt1.x, pt1.y, 1.0);
  const Eigen::Vector3d p2_0(pt2.x, pt2.y, 1.0);
  Eigen::Vector3d p1, p2;
  Eigen::Vector2d n1, n2, delta_p1, delta_p2, delta_p1_old, delta_p2_old;
  delta_p1_old << 0.0, 0.0;
  delta_p2_old << 0.0, 0.0;
  Eigen::Matrix<double, 2, 3> S;
  S << 1, 0, 0, 0, 1, 0;
  Eigen::Matrix2d essential_bar = essential.topLeftCorner(2, 2);
  double lambda;
  for (unsigned int i = 0; i < max_iterations; i++) {
    n1 = S * (essential * p2_old);
    n2 = S * (essential.transpose() * p1_old);
    lambda = ((p1_0.transpose() * essential * p2_0)(0) -
              (delta_p1_old.transpose() * essential_bar * delta_p2_old)(0)) /
             (n1.transpose() * n1 + n2.transpose() * n2)(0);
    delta_p1 = lambda * n1;
    delta_p2 = lambda * n2;
    p1 = p1_0 - (S.transpose() * delta_p1);
    p2 = p2_0 - (S.transpose() * delta_p2);
    p1_old = p1;
    p2_old = p2;
    delta_p1_old = delta_p1;
    delta_p2_old = delta_p2;
    // std::cout << "ptL_est: [" << p1.transpose() << "] ptR_est: [" <<
    // p2.transpose() << "]" << std::endl;
  }
  Eigen::Vector3d z = p1.cross(R * p2);
  Eigen::Vector3d X =
      ((z.transpose() * (essential * p2))(0) / (z.transpose() * z)(0)) * p1;
  return X;
}

Eigen::Vector3d lindstrom_triangulation(const cv::Point2d &pt1,
                                        const cv::Point2d &pt2,
                                        const Eigen::Matrix3d &essential,
                                        const Eigen::Matrix3d &R) {
  /*
          Optimal triangulation method for two cameras with parallel principal
     axes
          Based of off this paper by Peter Lindstrom:
     https://e-reports-ext.llnl.gov/pdf/384387.pdf  **Listing 2**
  */
  // std::cout << "ptL_noisy: " << pt1 << " ptR_noisy: " << pt2 << std::endl;
  const unsigned int max_iterations = 7;
  Eigen::Vector3d p1_old(pt1.x, pt1.y, 1.0);
  Eigen::Vector3d p2_old(pt2.x, pt2.y, 1.0);
  const Eigen::Vector3d p1_0(pt1.x, pt1.y, 1.0);
  const Eigen::Vector3d p2_0(pt2.x, pt2.y, 1.0);
  Eigen::Vector3d p1, p2;
  Eigen::Vector2d n1, n2, delta_p1, delta_p2;
  Eigen::Matrix<double, 2, 3> S;
  S << 1, 0, 0, 0, 1, 0;
  Eigen::Matrix2d essential_bar = essential.topLeftCorner(2, 2);
  double a, b, c, d, lambda;
  c = p1_0.transpose() * (essential * p2_0);
  for (unsigned int i = 0; i < max_iterations; i++) {
    n1 = S * (essential * p2_old);
    n2 = S * (essential.transpose() * p1_old);
    a = n1.transpose() * (essential_bar * n2);
    b = (0.5 * ((n1.transpose() * n1) + (n2.transpose() * n2)))(0);
    d = sqrt(b * b - a * c);
    double signum_b = (b > 0) ? 1 : ((b < 0) ? -1 : 0);
    lambda = c / (b + signum_b * d);
    delta_p1 = lambda * n1;
    delta_p2 = lambda * n2;
    p1 = p1_0 - (S.transpose() * delta_p1);
    p2 = p2_0 - (S.transpose() * delta_p2);
    p1_old = p1;
    p2_old = p2;
    // std::cout << "ptL_est: [" << p1.transpose() << "] ptR_est: [" <<
    // p2.transpose() << "]" << std::endl;
  }
  Eigen::Vector3d z = p1.cross(R * p2);
  Eigen::Vector3d X =
      ((z.transpose() * (essential * p2))(0) / (z.transpose() * z)(0)) * p1;
  return X;
}

ImageWithCameraInfo::ImageWithCameraInfo(
    sensor_msgs::ImageConstPtr _image_msg_ptr,
    sensor_msgs::CameraInfoConstPtr _info_msg_ptr)
    : image_msg_ptr(_image_msg_ptr), info_msg_ptr(_info_msg_ptr),
      image_time(_image_msg_ptr->header.stamp) {}

FrameHistory::FrameHistory(std::string img_topic, unsigned int hist_size)
    : topic_name(img_topic), history_size(hist_size), _image_transport(nh),
      frame_count(0) {
  std::stringstream console_msg;
  console_msg << "[FrameHistory] size set to " << history_size << std::endl
              << "\tSubscribing to image topic: " << topic_name;
  ROS_INFO(console_msg.str().c_str());
  _image_sub = _image_transport.subscribeCamera(
      img_topic, 1, &FrameHistory::image_callback, this);
  if (_image_sub.getNumPublishers() == 0) {
    std::stringstream error_msg;
    error_msg << "[FrameHistory] no publishers currently publishing to "
              << topic_name;
    ROS_WARN(error_msg.str().c_str());
  }
}

FrameHistory::~FrameHistory() {
  std::stringstream console_msg;
  console_msg << "[FrameHistory] Unsubscribed from image topic: " << topic_name
              << std::endl
              << "[FrameHistory] Deleting FrameHistory object" << std::endl;
  ROS_INFO(console_msg.str().c_str());
}

void FrameHistory::image_callback(
    const sensor_msgs::ImageConstPtr &image_msg,
    const sensor_msgs::CameraInfoConstPtr &info_msg) {
  /**
          Adds an  ImageWithCameraInfo object to the frame history ring buffer
  */
  ImageWithCameraInfo current_frame(image_msg, info_msg);
  bool full = _frame_history_ring_buffer.size() >= history_size;
  std::stringstream debug_msg;
  debug_msg << "Adding frame to ring buffer "
            << "[frame=" << frame_count << ","
            << "full=" << (full ? "true" : "false")
            << ",frames_available=" << _frame_history_ring_buffer.size() << "]"
            << std::endl;
  ROS_DEBUG(debug_msg.str().c_str());
  if (!full) {
    _frame_history_ring_buffer.push_back(current_frame);
  } else {
    _frame_history_ring_buffer[frame_count % history_size] = current_frame;
  }
  frame_count++;
}

std::vector<ImageWithCameraInfo>
FrameHistory::get_frame_history(unsigned int frames_requested) {
  /**
          Returns a vector with the last <num_frames> ImageWithCameraInfo
     objects
  */
  std::vector<ImageWithCameraInfo> frame_history;
  std::vector<ImageWithCameraInfo> sorted_frame_history =
      _frame_history_ring_buffer;
  if (_frame_history_ring_buffer.size() < frames_requested) {
    ROS_WARN("get_frame_history(%d): %d frames were requested, but there are "
             "%d frames available",
             frames_requested, frames_requested,
             _frame_history_ring_buffer.size());
  } else {
    std::sort(sorted_frame_history.begin(), sorted_frame_history.end());
    for (size_t i = 0; i < frames_requested; i++) {
      frame_history.push_back(sorted_frame_history[i]);
      if (i == frames_requested - 1)
        break;
    }
  }
  return frame_history;
}

} // namespace nav
