#pragma once

#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
  Creates a new kernel that is rotationally invariant by averaging rotated instances of the
  original.
  kernel - original kernel
  rotations - OPTIONAL. The number of the times that the original kernel will be rotated before
    the rotated versions are averaged together. The rotated kernels will be uniformly spread
    angularly.
*/
cv::Mat makeRotInvariant(const cv::Mat &kernel, int rotations=8);
