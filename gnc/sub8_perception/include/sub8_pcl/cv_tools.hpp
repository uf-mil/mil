#pragma once
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>

namespace sub {

typedef std::vector<cv::Point> Contour;

// Compute the centroid of an OpenCV contour (Not templated)
cv::Point contour_centroid(Contour& contour);
}