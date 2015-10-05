#include <eigen3/Eigen/Dense>
#include <math.h>

#include "util.h"

#ifndef __EIGEN_HELPER_H__
#define __EIGEN_HELPER_H__

#define EIGEN3COMP(v) (v).x(), (v).y(), (v).z()
#define EIGEN2COMP(v) (v).x(), (v).y()

namespace Eigen {

template <class Vector>
Vector ScalarCross(const Vector& v1, const Vector& v2) {
  return (v1.x() * v2.y() - v1.y() * v2.x());
}

template <typename T>
Matrix<T, 2, 1> Perp2(const Matrix<T, 2, 1>& v) {
  return (Matrix<T, 2, 1>(-v.y(), v.x()));
}

template <typename kScalarType, size_t kDim>
class MultiVariateNormal {
 protected:
  Eigen::Matrix<kScalarType, kDim, 1> scale_;
  Eigen::Matrix<kScalarType, kDim, kDim> rotation_;
  const Eigen::Matrix<kScalarType, kDim, 1> mean_;

 public:
  MultiVariateNormal(
      const Eigen::Matrix<kScalarType, kDim, 1>& mean,
      const Eigen::Matrix<kScalarType, kDim, kDim>& covariance) : mean_(mean) {
    // Perform eigenvalue decomposition of std_dev
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<kScalarType, kDim, kDim> >
        eigen_solver(covariance);
    rotation_ = eigen_solver.eigenvectors();
    scale_ = eigen_solver.eigenvalues();
    for (size_t i = 0; i < kDim; ++i) {
      scale_(i, 0) = std::sqrt(scale_(i,0));
    }
  }

  void Sample(Eigen::Matrix<kScalarType, kDim, 1>* sample_ptr) {
    Eigen::Matrix<kScalarType, kDim, 1>& sample = *sample_ptr;
    for (size_t i = 0; i < kDim; ++i) {
      sample(i,0) = randn<kScalarType>()*scale_(i,0);
    }
    sample = rotation_ * sample + mean_;
  }
};

}  // namespace Eigen

#endif  // __EIGEN_HELPER_H__
