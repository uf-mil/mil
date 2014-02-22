#ifndef GUARD_UIBAPTOZSJJMTCMG
#define GUARD_UIBAPTOZSJJMTCMG

#include <odom_estimator/util.h>

#include <boost/preprocessor/seq/fold_right.hpp>
#include <boost/fusion/adapted/struct/define_struct.hpp>
#include <boost/preprocessor/seq/for_each_product.hpp>
#include <boost/preprocessor/seq/push_front.hpp>
#include <boost/preprocessor/seq/push_back.hpp>
#include <boost/preprocessor/seq/pop_back.hpp>

namespace odom_estimator {


// A manifold is a mathematical space that can behave in any possible way
// globally, but locally must act like an Euclidean space of a certain
// dimension, D. Points on manifolds are made more amenable to analysis by
// only looking at D-dimensional Euclidean vector between points, obtained by
// subtracting points (operator-) and by adding a D-dimensional Euclidean
// vector to a point (operator+).
// This type acts like Eigen vectors so instances can be used interchangably
// with them.
template<typename Derived, int D>
struct IManifold {
  static int const RowsAtCompileTime = D;
  virtual unsigned int rows() const = 0;
  virtual Vec<D> operator-(Derived const &other) const = 0;
  virtual Derived operator+(Vec<D> const &other) const = 0;
};


// Macro magic to define a new manifold type that just combines multiple
// named manifold types

#define FOLD_LEFT_SKIPPING_FIRST(op, state, seq) \
  BOOST_PP_IF( \
    BOOST_PP_DEC(BOOST_PP_SEQ_SIZE(seq)), \
    BOOST_PP_SEQ_FOLD_LEFT(op, state, BOOST_PP_SEQ_TAIL(seq)), \
    state)

#define GENERATE_FIELD(R, ATTRIBUTE_TUPLE_SIZE, ATTRIBUTE) \
\
  BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,0,ATTRIBUTE) \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE);


#define myop(s, state, x) addRowsAtCompileTime(state, BOOST_PP_TUPLE_ELEM(2,0,x)::RowsAtCompileTime)
#define myop2(s, state, x) state + BOOST_PP_TUPLE_ELEM(2,1,x).rows()

#define GENERATE_ARG( \
    R, ATTRIBUTE_TUPLE_SIZE, I, ATTRIBUTE) \
\
    BOOST_PP_COMMA_IF(I) \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,0,ATTRIBUTE) \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE)

#define GENERATE_PARAM( \
    R, ATTRIBUTE_TUPLE_SIZE, I, ATTRIBUTE) \
\
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE), \

#define GENERATE_SETTER( \
    R, ATTRIBUTE_TUPLE_SIZE, I, ATTRIBUTE) \
\
    BOOST_PP_COMMA_IF(I) \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE)(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE))

#define GENERATE_SUBTRACTOR( \
    R, ATTRIBUTE_TUPLE_SIZE, I, ATTRIBUTE) \
\
    BOOST_PP_COMMA_IF(I) \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE) - other.BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE)

#define GENERATE_DEF( \
    R, ATTRIBUTES_SEQ, I, ATTRIBUTE) \
\
  unsigned int const BOOST_PP_CAT(BOOST_PP_TUPLE_ELEM(2,1,ATTRIBUTE),_start) = BOOST_PP_IF(I,BOOST_PP_CAT(BOOST_PP_TUPLE_ELEM(2,1,BOOST_PP_SEQ_ELEM(BOOST_PP_DEC(I), ATTRIBUTES_SEQ)),_start),0); \
  unsigned int const BOOST_PP_CAT(BOOST_PP_TUPLE_ELEM(2,1,ATTRIBUTE), _end) = BOOST_PP_CAT(BOOST_PP_TUPLE_ELEM(2,1,ATTRIBUTE),_start) + BOOST_PP_TUPLE_ELEM(2,1,ATTRIBUTE).rows();

#define GENERATE_ADDER( \
    R, ATTRIBUTE_TUPLE_SIZE, I, ATTRIBUTE) \
\
    BOOST_PP_COMMA_IF(I) \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE) + other.segment( \
      BOOST_PP_CAT(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE),_start), \
      BOOST_PP_CAT(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE),_end) - \
      BOOST_PP_CAT(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE),_start))

#define GENERATE_ASSERT(R, ATTRIBUTE_TUPLE_SIZE, ATTRIBUTE) \
\
  assert(BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE) == other.BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE));

#define ODOM_ESTIMATOR_DEFINE_MANIFOLD_IMPL( \
    NAME, TYPE_ATTRIBUTES_SEQ, ATTRIBUTES_SEQ, ATTRIBUTE_TUPLE_SIZE) \
\
    struct NAME { \
\
        BOOST_PP_SEQ_FOR_EACH_R( \
            1, \
            GENERATE_FIELD, \
            ATTRIBUTE_TUPLE_SIZE, \
            BOOST_PP_SEQ_TAIL(TYPE_ATTRIBUTES_SEQ)) \
\
        BOOST_PP_SEQ_FOR_EACH_R( \
            1, \
            GENERATE_FIELD, \
            ATTRIBUTE_TUPLE_SIZE, \
            BOOST_PP_SEQ_TAIL(ATTRIBUTES_SEQ)) \
\
        static int const RowsAtCompileTime = \
            FOLD_LEFT_SKIPPING_FIRST(myop, 0, ATTRIBUTES_SEQ); \
\
        unsigned int rows() const { \
            return FOLD_LEFT_SKIPPING_FIRST(myop2, 0, ATTRIBUTES_SEQ); \
        } \
\
        Vec<RowsAtCompileTime> \
        operator-(NAME const &other) const { \
          BOOST_PP_SEQ_FOR_EACH_R( \
              1, \
              GENERATE_ASSERT, \
              ATTRIBUTE_TUPLE_SIZE, \
              BOOST_PP_SEQ_TAIL(TYPE_ATTRIBUTES_SEQ)) \
          return (Vec<RowsAtCompileTime>(rows()) << \
            BOOST_PP_SEQ_FOR_EACH_I_R(1, \
              GENERATE_SUBTRACTOR, \
              ATTRIBUTE_TUPLE_SIZE, \
              BOOST_PP_SEQ_TAIL(ATTRIBUTES_SEQ)) \
          ).finished(); \
        } \
        NAME operator+(const Vec<RowsAtCompileTime> &other) const { \
          BOOST_PP_SEQ_FOR_EACH_I_R(1, \
            GENERATE_DEF, \
            BOOST_PP_SEQ_TAIL(ATTRIBUTES_SEQ), \
            BOOST_PP_SEQ_TAIL(ATTRIBUTES_SEQ)) \
          return NAME( \
            BOOST_PP_SEQ_FOR_EACH_I_R(1, \
              GENERATE_PARAM, \
              ATTRIBUTE_TUPLE_SIZE, \
              BOOST_PP_SEQ_TAIL(TYPE_ATTRIBUTES_SEQ)) \
            BOOST_PP_SEQ_FOR_EACH_I_R(1, \
              GENERATE_ADDER, \
              ATTRIBUTE_TUPLE_SIZE, \
              BOOST_PP_SEQ_TAIL(ATTRIBUTES_SEQ)) \
          ); \
        } \
\
        NAME(BOOST_PP_SEQ_FOR_EACH_I_R(1, \
                                       GENERATE_ARG, \
                                       ATTRIBUTE_TUPLE_SIZE, \
                                       BOOST_PP_SEQ_TAIL(TYPE_ATTRIBUTES_SEQ) \
                                       BOOST_PP_SEQ_TAIL(ATTRIBUTES_SEQ))) \
          : BOOST_PP_SEQ_FOR_EACH_I_R(1, \
                                      GENERATE_SETTER, \
                                      ATTRIBUTE_TUPLE_SIZE, \
                                      BOOST_PP_SEQ_TAIL(TYPE_ATTRIBUTES_SEQ) \
                                      BOOST_PP_SEQ_TAIL(ATTRIBUTES_SEQ)) { }

#define APPEND_IF_SEQ_LENGTH_IS_1(SEQ, ITEM) \
  BOOST_PP_IF( \
    BOOST_PP_DEC(BOOST_PP_SEQ_SIZE(SEQ)), \
    SEQ, \
    BOOST_PP_SEQ_PUSH_BACK(SEQ, ITEM))

#define ODOM_ESTIMATOR_DEFINE_MANIFOLD_BEGIN(NAME, TYPE_ATTRIBUTES, ATTRIBUTES) \
    ODOM_ESTIMATOR_DEFINE_MANIFOLD_IMPL( \
        NAME, \
        BOOST_PP_CAT(BOOST_FUSION_ADAPT_STRUCT_FILLER_0(0,0)TYPE_ATTRIBUTES,_END), \
        APPEND_IF_SEQ_LENGTH_IS_1(BOOST_PP_CAT(BOOST_FUSION_ADAPT_STRUCT_FILLER_0(0,0)ATTRIBUTES,_END), (Vec<0>, _dummy)), \
        2)

#define ODOM_ESTIMATOR_DEFINE_MANIFOLD_END() };


template<typename First, typename Second>
ODOM_ESTIMATOR_DEFINE_MANIFOLD_BEGIN(ManifoldPair,
  ,
  (First, first)
  (Second, second)
)
ODOM_ESTIMATOR_DEFINE_MANIFOLD_END()

ODOM_ESTIMATOR_DEFINE_MANIFOLD_BEGIN(EmptyTestManifold,
  ,
)
ODOM_ESTIMATOR_DEFINE_MANIFOLD_END()


// is an IManifold type for unit quaternions.
// If the quaternion represents the rotation of a vector from the body frame
// into the world frame, as orientations are usually represented, this class
// will result in the orientation's covariance being represented in the
// world frame.
struct QuaternionManifold : public IManifold<QuaternionManifold, 3>, public Quaternion {
  QuaternionManifold(Quaternion q) :
    Quaternion(q.normalized()) {
  }
  unsigned int rows() const {
    return RowsAtCompileTime;
  }
  Vec<RowsAtCompileTime> operator-(QuaternionManifold const &other) const {
    return rotvec_from_quat(*this * other.conjugate());
  }
  QuaternionManifold operator+(Vec<RowsAtCompileTime> const &other) const {
    return quat_from_rotvec(other) * *this;
  }
  operator Quaternion() const {
    return *this;
  }
};

struct WrappedScalar : public IManifold<WrappedScalar, 1> {
private:
  double s;
public:
  WrappedScalar(double s) :
    s(s) {
  }
  unsigned int rows() const {
    return RowsAtCompileTime;
  }
  Vec<RowsAtCompileTime> operator-(WrappedScalar const &other) const {
    return (Vec<1>() << s - other.s).finished();
  }
  WrappedScalar operator+(Vec<RowsAtCompileTime> const &other) const {
    return WrappedScalar(s + other(0));
  }
  operator double() const {
    return s;
  }
};


}

#endif
