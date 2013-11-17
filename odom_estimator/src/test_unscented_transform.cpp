#include <iostream>

#include "odom_estimator/unscented_transform.h"

using namespace Eigen;
using namespace odom_estimator;

Vector4d func(Vector3d p) {
  return Vector4d(exp(p(1)), p(0), -p(2), p(1) + p(2)) + Vector4d(1, 2, 4, 3);
}

template<class T>
void print(const T &res) {
  std::cout << res.mean.transpose() << std::endl;
  std::cout << std::endl;
  std::cout << res.cov << std::endl;
  std::cout << std::endl;
  std::cout << res.cross_cov << std::endl;
}

int main() {
  Vector3d initial(1, 2, 3);
  Matrix3d cov = (Matrix3d() <<
    1, 0, 0,
    0, 1, 0,
    0, 0, 1).finished();
  
  UnscentedTransform<Vector4d, 4, Vector3d, 3> res1(func, initial, cov);
  std::cout << "STATIC" << std::endl;
  print(res1);
  
  std::cout << std::endl;
  
  UnscentedTransform<VectorXd, Dynamic, VectorXd, Dynamic> res2(func, initial, cov);
  std::cout << "DYNAMIC" << std::endl;
  print(res2);
  
  return 0;
}
