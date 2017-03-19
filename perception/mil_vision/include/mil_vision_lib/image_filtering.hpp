#pragma once

#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <mil_tools/mil_tools.hpp>

namespace nav
{

/*
  Returns a version of the input kernel rotated about its center point.
  kernel - original kernel
  theta - anlgle in radians by which to rotate the kernel. Positive angles --> counterclockwise.
  deg - OPTIONAL. Will assume that theta is given in degrees if set to true.
  no_expand - OPTIONAL. Will leave the output size the same as the input size. Parts of the
    original kernel may fall outside the output canvas after the rotation.
*/
cv::Mat rotateKernel(const cv::Mat &kernel, float theta, bool deg=false, bool no_expand=false);

/*
  Creates a new kernel that is rotationally invariant by averaging rotated instances of the
  original.
  kernel - original kernel
  rotations - OPTIONAL. The number of the times that the original kernel will be rotated before
    the rotated versions are averaged together. The rotated kernels will be uniformly spread
    angularly.
*/
cv::Mat makeRotInvariant(const cv::Mat &kernel, int rotations=8);

/*
  Returns the minimum theta for which a version of the kernel that has been rotated
  by theta radians will be approximately identical to the original.
  kernel - input kernel
  ang_res - OPTIONAL. The result will be a multiple of this angle
  deg - OPTIONAL. The output will be in degrees instead of radias if set to true
*/
float getRadialSymmetryAngle(const cv::Mat &kernel, float ang_res=0.1, bool deg=false);

} // namespace nav
