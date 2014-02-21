#ifndef GUARD_UIBAPTOZSJJMTCMG
#define GUARD_UIBAPTOZSJJMTCMG

#include <odom_estimator/util.h>

#include <boost/preprocessor/seq/fold_right.hpp>
#include <boost/fusion/adapted/struct/define_struct.hpp>

namespace odom_estimator {

static inline constexpr int addRowsAtCompileTime(int a, int b) {
  return a == Dynamic ? Dynamic :
         b == Dynamic ? Dynamic :
         a + b;
}

#define FOLD_LEFT_SKIPPING_FIRST(op, state, seq) \
  BOOST_PP_IF( \
    BOOST_PP_DEC(BOOST_PP_SEQ_SIZE(seq)), \
    BOOST_PP_SEQ_FOLD_LEFT(op, state, BOOST_PP_SEQ_TAIL(seq)), \
    state)
#define FOR_EACH_I_SKIPPING_FIRST(macro, data, seq) \
  BOOST_PP_IF( \
    BOOST_PP_DEC(BOOST_PP_SEQ_SIZE(seq)), \
    BOOST_PP_SEQ_FOR_EACH_I_R(1, macro, data, BOOST_PP_SEQ_TAIL(seq)), \
    BOOST_PP_EMPTY())

#define BOOST_FUSION_DEFINE_STRUCT_ATTR_I(R, ATTRIBUTE_TUPLE_SIZE, ATTRIBUTE)   \
                                                                                \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,0,ATTRIBUTE)                       \
        BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE);

#define BOOST_FUSION_DEFINE_STRUCT_SEQ_DEFAULT_CTOR_FILLER_I(                   \
    R, ATTRIBUTE_TUPLE_SIZE, I, ATTRIBUTE)                                      \
                                                                                \
    BOOST_PP_COMMA_IF(I)                                                        \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE)()

#define BOOST_FUSION_DEFINE_STRUCT_COPY_CTOR_FILLER_I(                          \
    R, ATTRIBUTE_TUPLE_SIZE, I, ATTRIBUTE)                                      \
                                                                                \
    BOOST_PP_COMMA_IF(I)                                                        \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE)(                      \
        other_self.BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE))

#define myop(s, state, x) addRowsAtCompileTime(state, BOOST_PP_TUPLE_ELEM(2,0,x)::RowsAtCompileTime)
#define myop2(s, state, x) state + BOOST_PP_TUPLE_ELEM(2,1,x).rows()

#define GENERATE_ARG(                   \
    R, ATTRIBUTE_TUPLE_SIZE, I, ATTRIBUTE)                                      \
                                                                                \
    BOOST_PP_COMMA_IF(I)                                                        \
    BOOST_PP_TUPLE_ELEM(ATTRIBUTE_TUPLE_SIZE,1,ATTRIBUTE)()

#define BOOST_FUSION_DEFINE_STRUCT_IMPL(                                        \
    NAMESPACE_SEQ, NAME, ATTRIBUTES_SEQ, ATTRIBUTE_TUPLE_SIZE)                  \
                                                                                \
    BOOST_FUSION_ADAPT_STRUCT_NAMESPACE_DEFINITION_BEGIN(NAMESPACE_SEQ)         \
                                                                                \
    struct NAME                                                                 \
    {                                                                           \
        typedef NAME self_type;                                                 \
                                                                                \
        BOOST_PP_SEQ_FOR_EACH_R(                                                    \
            1,                                                                      \
            BOOST_FUSION_DEFINE_STRUCT_ATTR_I,                                      \
            ATTRIBUTE_TUPLE_SIZE,                                                   \
            ATTRIBUTES_SEQ)                                                         \
                                                                                    \
        static int const RowsAtCompileTime =                                        \
            FOLD_LEFT_SKIPPING_FIRST(myop, 0, ATTRIBUTES_SEQ);                       \
                                                                                    \
        unsigned int rows() const {                                                 \
            return FOLD_LEFT_SKIPPING_FIRST(myop2, 0, ATTRIBUTES_SEQ);               \
        }                                                                           \
                                                                                    \
        Vec<RowsAtCompileTime>()                                                    \
        operator-(self_type const &other) const {       \
          return (Vec<RowsAtCompileTime>() <<                                                    \
            first - other.first,                                                    \
            second - other.second).finished();                                      \
        }                                                                           \
        self_type operator+(const DeltaType &other) const {       \
          return self_type(                                       \
            first + other.head(first.rows()),                                       \
            second + other.tail(second.rows()));                                    \
        }                                                                           \
                                                                                    \
        NAME(self_type const& other_self)                                           \
          : BOOST_PP_SEQ_FOR_EACH_I_R(                                              \
                1,                                                                  \
                BOOST_FUSION_DEFINE_STRUCT_COPY_CTOR_FILLER_I,                      \
                ATTRIBUTE_TUPLE_SIZE,                                               \
                ATTRIBUTES_SEQ)                                                     \
        {} \
                                                                                \
        BOOST_FUSION_DEFINE_STRUCT_CTOR(                                        \
            NAME, ATTRIBUTES_SEQ, ATTRIBUTE_TUPLE_SIZE) \
        \
    };                                                                          \
                                                                                \
    BOOST_FUSION_ADAPT_STRUCT_NAMESPACE_DEFINITION_END(NAMESPACE_SEQ)


#define BOOST_FUSION_DEFINE_STRUCT(NAMESPACE_SEQ, NAME, ATTRIBUTES)             \
    BOOST_FUSION_DEFINE_STRUCT_IMPL(                                            \
        (0)NAMESPACE_SEQ,                                                       \
        NAME,                                                                   \
        BOOST_PP_CAT(BOOST_FUSION_ADAPT_STRUCT_FILLER_0(0,0)ATTRIBUTES,_END),   \
        2)


//        newnew \
//        NAME(FOR_EACH_I_SKIPPING_FIRST(GENERATE_ARG, ATTRIBUTE_TUPLE_SIZE, ATTRIBUTES_SEQ)) { } \
//        endnewnew \

BOOST_FUSION_DEFINE_STRUCT(
  (odom_estimator),
  NewManifoldPair,
  (Vec<3>, first)
  (Vec<3>, second)
  )

BOOST_FUSION_DEFINE_STRUCT(
  (odom_estimator),
  NewManifoldPair,
  
  )

START

FOLD_LEFT_SKIPPING_FIRST(myop, 0, ((a, b))((c, d))((e, f)))

END



a
BOOST_FUSION_ADAPT_STRUCT_FILLER_0(0,0)
b

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
