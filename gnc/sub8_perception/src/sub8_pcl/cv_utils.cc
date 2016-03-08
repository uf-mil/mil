#include <sub8_pcl/cv_tools.hpp>

namespace sub {
cv::Point contour_centroid(Contour& contour) {
  cv::Moments m = cv::moments(contour, true);
  cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
  return center;
}
}