#include "drake/common/autodiff.h"
#include "drake/math/expmap.h"
#include "drake/math/quaternion.h"

#include <vector>

namespace cassie {
namespace filter {

using drake::math::quatConjugate;
using drake::math::expmap2quat;
const Eigen::Vector3d g = {0, 0, -9.81};

template <int N> using Vector = Eigen::Matrix<double, N, 1>;

template <int N, int M>
using AutoDiffVector =
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, M, 1>>, N, 1>;

template <int N>
Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, N, 1>>, N, 1>
initializeAutoDiff(Eigen::Matrix<double, N, 1> x) {
  using Scalar = Eigen::AutoDiffScalar<Eigen::Matrix<double, N, 1>>;
  Eigen::Matrix<Scalar, N, 1> x_ad;
  for (int i = 0; i < N; i++) {
    x_ad(i) = Scalar(x(i), N, i);
  }
  return x_ad;
}

template <int N>
Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, N, 1>>, N, 1>
resetAutoDiff(Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, N, 1>>,
                            N, 1> &x) {
  for (int i = 0; i < N; i++) {
    x(i).derivatives() = Eigen::Matrix<double, N, 1>::Zero();
    x(i).derivatives()(i) = 1;
  }
  return x;
}

template <int N, int M>
Eigen::Matrix<double, N, M> autoDiffToGradientMatrix(AutoDiffVector<N, M> &x) {
  Eigen::Matrix<double, N, M> J;
  for (int i = 0; i < N; i++) {
    J.row(i) = x(i).derivatives().transpose();
  }
  return J;
}

template <typename Derived1, typename Derived2>
Eigen::Matrix<typename Derived1::Scalar, 4, 1>
quatProduct(const Eigen::MatrixBase<Derived1> &q1,
            const Eigen::MatrixBase<Derived2> &q2) {
  Eigen::Matrix<typename Derived1::Scalar, 4, 1> q;

  auto q1_vec = q1.template tail<3>();
  auto q2_vec = q2.template tail<3>();

  q(0) = q1(0) * q2(0) - q1_vec.dot(q2_vec);
  q.template tail<3>() = q1(0) * q2_vec + q2(0) * q1_vec + q1_vec.cross(q2_vec);

  return q;
}

template <typename Derived1, typename Derived2>
Eigen::Matrix<typename Derived1::Scalar, 3, 1>
quatRotateVec(const Eigen::MatrixBase<Derived1> &q,
              const Eigen::MatrixBase<Derived2> &v) {
  Eigen::Matrix<typename Derived2::Scalar, 4, 1> v_;
  v_.template tail<3>() = -v;
  v_(0) = 0;

  return quatProduct(q, quatConjugate(quatProduct(q, -v))).template tail<3>();
}

/*
Helper methods to provide a view into relevant parts of a filter state
*/
template <typename Scalar, int Nx>
Eigen::Map<Eigen::Matrix<Scalar, 3, 1>>
get_filter_pos(Eigen::Matrix<Scalar, Nx, 1> &x) {
  return Eigen::Map<Eigen::Matrix<Scalar, 3, 1>>(x.template head<3>().data());
}

template <typename Scalar, int Nx>
Eigen::Map<Eigen::Matrix<Scalar, 4, 1>>
get_filter_quat(Eigen::Matrix<Scalar, Nx, 1> &x) {
  return Eigen::Map<Eigen::Matrix<Scalar, 4, 1>>(
      x.template segment<4>(3).data());
}

template <typename Scalar, int Nx>
Eigen::Map<Eigen::Matrix<Scalar, 3, 1>>
get_filter_vel(Eigen::Matrix<Scalar, Nx, 1> &x) {
  return Eigen::Map<Eigen::Matrix<Scalar, 3, 1>>(
      x.template segment<3>(7).data());
}

template <typename Scalar, int Nx>
Eigen::Map<Eigen::Matrix<Scalar, 3, 1>>
get_filter_foot_pos(Eigen::Matrix<Scalar, Nx, 1> &x, int foot_idx) {
  return Eigen::Map<Eigen::Matrix<Scalar, 3, 1>>(
      x.template segment<3>(10 + 7 * foot_idx).data());
}

template <typename Scalar, int Nx>
Eigen::Map<Eigen::Matrix<Scalar, 4, 1>>
get_filter_foot_quat(Eigen::Matrix<Scalar, Nx, 1> &x, int foot_idx) {
  return Eigen::Map<Eigen::Matrix<Scalar, 4, 1>>(
      x.template segment<4>(13 + 7 * foot_idx).data());
}

/*
Process update for a floating base with an IMU
*/
template <typename Scalar, int N>
void process_update(double dt, Eigen::Matrix<Scalar, N, 1> &x,
                    const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel) {
  auto p = get_filter_pos(x);
  auto q = get_filter_quat(x);
  auto v = get_filter_vel(x);

  // v += dt*(accel + quatRotateVec(quatConjugate(q),g));
  p += dt * quatRotateVec(q, v);
  Eigen::Vector4d qw = expmap2quat(gyro * dt);
  q = quatProduct(q, qw);
}

/*
Dynamics update for the filter
*/
template <int N>
void dynamics_update(double dt, AutoDiffVector<N, N> &x,
                     Eigen::Matrix<double, N, N> &P,
                     const Eigen::Matrix<double, N, N> &Q,
                     const Eigen::Vector3d &gyro,
                     const Eigen::Vector3d &accel) {
  resetAutoDiff(x);
  process_update(dt, x, gyro, accel);
  auto A = autoDiffToGradientMatrix(x);
  P = A * P * A.transpose() + Q;
}

template <typename Scalar, int N>
Eigen::Matrix<Scalar, 7, 1> filter_measurement(Eigen::Matrix<Scalar, N, 1> &x,
                                               int foot_idx) {
  // extract relevant quantities
  auto p = get_filter_pos(x);
  auto q = get_filter_quat(x);
  auto p_foot = get_filter_foot_pos(x, foot_idx);
  auto q_foot = get_filter_foot_quat(x, foot_idx);

  // compute measurements
  auto p_foot_body = quatRotateVec(quatConjugate(q), p_foot - p);
  auto q_foot_body = quatProduct(quatConjugate(q), q_foot);
  Eigen::Matrix<Scalar, 7, 1> meas;
  meas << p_foot_body, q_foot_body;
  return meas;
}

template <typename Scalar, int N, int Ny>
void ekf_measurement_update(Eigen::Matrix<Scalar, N, 1> &x,
                            Eigen::Matrix<double, N, N> &P,
                            const Eigen::Matrix<double, Ny, 1> y,
                            const Eigen::Matrix<Scalar, Ny, 1> h,
                            const Eigen::Matrix<double, Ny, N> H,
                            const Eigen::Matrix<double, Ny, Ny> R) {
  auto S = H * P * H.transpose() + R;
  auto K = P * H.transpose() * S.inverse();
  x += K * (y - h);
  P = (Eigen::Matrix<double, N, N>::Identity() - K * H) * P;
}

template <int N>
void kinematic_measurement_update(
    AutoDiffVector<N, N> &x, Eigen::Matrix<double, N, N> &P,
    std::vector<bool> &contact,
    const std::vector<Eigen::Matrix<double, 7, 1>> &y,
    const std::vector<Eigen::Matrix<double, 7, 7>> &R) {
  AutoDiffVector<7, N> h;
  Eigen::Matrix<double, 7, N> H;

  // assume the contact errors are independent
  for (uint i = 0; i < y.size(); i++) {
    if (!contact[i]) {
      break;
    }
    resetAutoDiff(x);
    h = filter_measurement(x, i);
    H = autoDiffToGradientMatrix(h);
    ekf_measurement_update(x, P, y[i], h, H, R[i]);
  }
}
}  // namespace filter
}  // namespace cassie
