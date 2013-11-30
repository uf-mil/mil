#ifndef GUARD_HVRLSXDXNEOHTCRV
#define GUARD_HVRLSXDXNEOHTCRV

namespace odom_estimator {
namespace magnetic {



Vec<3> getMagneticField(Vec<3> pos_eci) {
    return Vec<3>(0, 0, 50000e-9);
}



}
}

#endif
