#ifndef GUARD_UIBAPTOZSJJMTCMG
#define GUARD_UIBAPTOZSJJMTCMG

#include <odom_estimator/util.h>

namespace odom_estimator {



template<typename First, typename Second>
class ManifoldPair {
public:
  static int const RowsAtCompileTime = 
    First::RowsAtCompileTime == Dynamic ? Dynamic :
    Second::RowsAtCompileTime == Dynamic ? Dynamic :
    First::RowsAtCompileTime + Second::RowsAtCompileTime;
private:
  typedef Vec<RowsAtCompileTime> DeltaType;
  typedef SqMat<RowsAtCompileTime> CovType;
public:
  First first;
  Second second;
  ManifoldPair(First const &first, Second const &second) :
    first(first), second(second) {
  }
  
  unsigned int rows() const {
    return first.rows() + second.rows();
  }
  DeltaType operator-(const ManifoldPair<First, Second> &other) const {
    return (DeltaType() <<
      first - other.first,
      second - other.second).finished();
  }
  ManifoldPair<First, Second> operator+(const DeltaType &other) const {
    return ManifoldPair<First, Second>(
      first + other.head(first.rows()),
      second + other.tail(second.rows()));
  }
  
  static CovType build_cov(
      SqMat<First::RowsAtCompileTime> const &first_cov,
      SqMat<Second::RowsAtCompileTime> const &second_cov) {
    CovType res = CovType::Zero(first_cov.rows() + second_cov.rows(),
                                first_cov.rows() + second_cov.rows());
    res.topLeftCorner(first_cov.rows(), first_cov.rows()) = first_cov;
    res.bottomRightCorner(second_cov.rows(), second_cov.rows()) = second_cov;
    return res;
  }
};


}

#endif
