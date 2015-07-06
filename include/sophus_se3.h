#ifndef SOPHUS_SE3
#define SOPHUS_SE3

#include <sophus/se3.hpp>

namespace sophus_se3 {

////from https://github.com/adrelino/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
struct SophusSE3Plus{
    template<typename T>
    bool operator()(const T* x_raw, const T* delta_raw, T* x_plus_delta_raw) const {
        const Eigen::Map< const Sophus::SE3Group<T> > x(x_raw);
        const Eigen::Map< const Eigen::Matrix<T,6,1> > delta(delta_raw);
        Eigen::Map< Sophus::SE3Group<T> > x_plus_delta(x_plus_delta_raw);
        x_plus_delta = x * Sophus::SE3Group<T>::exp(delta);
        return true;
      }
};

#endif // SOPHUS_SE3

