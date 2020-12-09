#include "cassie/filter/kinematic_filter.h"
// #include "cassie/filter/filter_params.h"
#include <gtest/gtest.h>

namespace cassie {
namespace filter {
namespace {
using drake::math::autoDiffToValueMatrix;

// AutoDiff helpers
GTEST_TEST(AutoDiff, Reset) {
  Eigen::Matrix<double, 24, 1> x_ = Eigen::Matrix<double, 24, 1>::Constant(1);
  auto x = initializeAutoDiff(x_);

  x *= 2;

  for (int i = 0; i < x.size(); i++) {
    for (int j = 0; j < x(i).derivatives().size(); j++) {
      EXPECT_EQ(x(i).derivatives()(j), i == j ? 2.0 : 0.0);
    }
  }

  resetAutoDiff(x);

  for (int i = 0; i < x.size(); i++) {
    for (int j = 0; j < x(i).derivatives().size(); j++) {
      EXPECT_EQ(x(i).derivatives()(j), i == j ? 1.0 : 0.0);
    }
  }
}

GTEST_TEST(AutoDiff, GradientMatrix) {
  const int size = 3;
  Eigen::Matrix<double, size, 1> x_ =
      Eigen::Matrix<double, size, 1>::Constant(1);
  auto x = initializeAutoDiff(x_);

  x *= 2;
  Eigen::Matrix<double, size, size> J = autoDiffToGradientMatrix(x);

  EXPECT_EQ(J.rows(), size);
  EXPECT_EQ(J.cols(), size);
  for (int i = 0; i < J.rows(); i++) {
    for (int j = 0; j < J.cols(); j++) {
      EXPECT_EQ(J(i, j), i == j ? 2.0 : 0.0);
    }
  }
}

// Quaternion helpers

GTEST_TEST(Quaternion, QuatProduct) {
  Eigen::Vector4d q(.5, .5, .5, .5);
  Eigen::Vector4d p(.5, -.5, .5, .5);

  auto r = quatProduct(q, p);

  double eps = 1e-8;
  EXPECT_NEAR(r(0), 0.0, eps);
  EXPECT_NEAR(r(1), 0.0, eps);
  EXPECT_NEAR(r(2), 0.0, eps);
  EXPECT_NEAR(r(3), 1.0, eps);
}

GTEST_TEST(Quaternion, QuatRotateVec) {
  Eigen::Vector4d q;
  q << .5, .5, .5, .5;
  Eigen::Vector3d e1(1, 0, 0);
  Eigen::Vector3d e2(0, 1, 0);
  Eigen::Vector3d e3(0, 0, 1);

  auto r1 = quatRotateVec(q, e1);
  auto r2 = quatRotateVec(q, e2);
  auto r3 = quatRotateVec(q, e3);

  double eps = 1e-8;
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(r1(i), e2(i), eps);
    EXPECT_NEAR(r2(i), e3(i), eps);
    EXPECT_NEAR(r3(i), e1(i), eps);
  }
}

// Kinematic Filter functions

GTEST_TEST(KinematicFilter, ModifyState) {
  Eigen::Matrix<double, 24, 1> x_ = Eigen::Matrix<double, 24, 1>::Constant(1);
  auto x = initializeAutoDiff(x_);

  auto pos = get_filter_pos(x);
  auto quat = get_filter_quat(x);
  auto vel = get_filter_vel(x);
  auto foot_pos_0 = get_filter_foot_pos(x, 0);
  auto foot_pos_1 = get_filter_foot_pos(x, 1);
  auto foot_quat_0 = get_filter_foot_quat(x, 0);
  auto foot_quat_1 = get_filter_foot_quat(x, 1);

  pos *= 2;
  quat *= 3;
  vel *= 4;
  foot_pos_0 *= 5;
  foot_quat_0 *= 6;
  foot_pos_1 *= 7;
  foot_quat_1 *= 8;

  Eigen::VectorXd x_expect(24);
  x_expect << 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8,
      8, 8;

  for (int i = 0; i < 24; i++) {
    EXPECT_EQ(x(i), x_expect(i));
  }
}

GTEST_TEST(KinematicFilter, EKFMeasurementUpdate) {
  Eigen::Vector3d x = Eigen::Vector3d::Constant(1.0);
  Eigen::Matrix3d P = Eigen::Matrix3d::Identity();

  Eigen::Vector2d y = Eigen::Vector2d::Zero();
  Eigen::Vector2d h = x.head<2>();
  Eigen::Matrix<double, 2, 3> H = Eigen::Matrix<double, 2, 3>::Identity();
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();

  ekf_measurement_update(x, P, y, h, H, R);

  std::cout << x << std::endl;
  std::cout << P << std::endl;

  double eps = 1e-6;
  EXPECT_NEAR(x(0), 0.5, eps);
  EXPECT_NEAR(x(1), 0.5, eps);
  EXPECT_NEAR(x(2), 1.0, eps);

  EXPECT_NEAR(P(0, 0), 0.5, eps);
  EXPECT_NEAR(P(1, 1), 0.5, eps);
  EXPECT_NEAR(P(2, 2), 1.0, eps);
}

Eigen::Matrix<double, 24, 1> getStartState() {
  Eigen::Matrix<double, 24, 1> x;
  x << 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0;
  return x;
}

GTEST_TEST(KinematicFilter, ProcessUpdate) {
  Eigen::Matrix<double, 24, 1> x = getStartState();
  Eigen::Vector3d gyro(1, 1, 1);
  Eigen::Vector3d accel(1, 2, 3 + 9.81);

  process_update(0.5, x, gyro, accel);

  Eigen::VectorXd x_expect(24);
  x_expect << 1.5, 1.5, 1.5, 0.907706, 0.24226, 0.24226, 0.24226, 1, 1, 1, 1, 1,
      1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0;
  // Switch to this one if accelerometer data included in filter
  // x_expect << 1.5, 1.5, 1.5, 0.907706, 0.24226, 0.24226, 0.24226,
  //             1.5, 2, 2.5, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0;

  double eps = 1e-6;
  for (int i = 0; i < 24; i++) {
    EXPECT_NEAR(x(i), x_expect(i), eps);
  }
}

GTEST_TEST(KinematicFilter, DynamicsUpdate) {
  auto x = initializeAutoDiff(getStartState());
  Eigen::Matrix<double, 24, 24> P = Eigen::Matrix<double, 24, 24>::Identity();
  Eigen::Matrix<double, 24, 24> Q = Eigen::Matrix<double, 24, 24>::Identity();
  Eigen::Vector3d gyro(1, 1, 1);
  Eigen::Vector3d accel(1, 2, 3 + 9.81);

  dynamics_update(0.5, x, P, Q, gyro, accel);
  Eigen::Matrix<double, 24, 1> x_val = autoDiffToValueMatrix(x);

  Eigen::VectorXd x_expect(24);
  x_expect << 1.5, 1.5, 1.5, 0.907706, 0.24226, 0.24226, 0.24226, 1, 1, 1, 1, 1,
      1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0;
  // Switch to this one if accelerometer data included in filter
  // x_expect << 1.5, 1.5, 1.5, 0.907706, 0.24226, 0.24226, 0.24226,
  //             1.5, 2, 2.5, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0;

  double eps = 1e-6;
  for (int i = 0; i < 24; i++) {
    EXPECT_NEAR(x_val(i), x_expect(i), eps);
  }
  // TODO: Add test of covariance update
}

GTEST_TEST(KinematicFilter, FilterMeasurement) {
  Eigen::Matrix<double, 24, 1> x = getStartState();
  get_filter_foot_pos(x, 0) *= 2;
  get_filter_foot_pos(x, 1) *= 3;

  auto lfoot = filter_measurement(x, 0);
  auto rfoot = filter_measurement(x, 1);

  Eigen::VectorXd lfoot_expect(7);
  lfoot_expect << 1, 1, 1, 1, 0, 0, 0;
  Eigen::VectorXd rfoot_expect(7);
  rfoot_expect << 2, 2, 2, 1, 0, 0, 0;

  for (int i = 0; i < 7; i++) {
    EXPECT_EQ(lfoot(i), lfoot_expect(i));
  }
  for (int i = 0; i < 7; i++) {
    EXPECT_EQ(rfoot(i), rfoot_expect(i));
  }
}

GTEST_TEST(KinematicFilter, KinematicMeasurementUpdate) {
  // TODO: Add this test
}

}  // namespace
}  // namespace filter
}  // namespace cassie
