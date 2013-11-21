#include <iostream>

#include "odom_estimator/unscented_transform.h"

using namespace odom_estimator;

Vec<4> func(Vec<3> p) {
  return Vec<4>(exp(p(1)), p(0), -p(2), p(1) + p(2)) + Vec<4>(1, 2, 4, 3);
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
  Vec<3> initial(1, 2, 3);
  SqMat<3> cov = (SqMat<3>() <<
    1, 0, 0,
    0, 1, 0,
    0, 0, 1).finished();
  
  UnscentedTransform<Vec<4>, Vec<3> > res1(func, initial, cov);
  std::cout << "STATIC" << std::endl;
  print(res1);
  
  std::cout << std::endl;
  
  UnscentedTransform<Vec<Dynamic>, Vec<Dynamic> > res2(func, initial, cov);
  std::cout << "DYNAMIC" << std::endl;
  print(res2);
  
  return 0;
}
