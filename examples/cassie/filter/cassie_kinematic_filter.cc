#include "cassie/filter/cassie_kinematic_filter.h"
// #include "cassie/filter/kinematic_filter.h"
#include "drake/math/autodiff.h"
#include <algorithm>

#include <memory>
#include <utility>
#include <vector>

namespace cassie {
namespace filter {

using drake::math::autoDiffToValueMatrix;

CassieKinematicFilter::CassieKinematicFilter(
    std::unique_ptr<RigidBodyTree<double>> tree, KinematicsCache<double> cache,
    JointFilterParams joint_filter_params,
    ContactTriggerParams contact_trigger_params,
    KinematicFilterParams kinematic_filter_params)
    : tree(std::move(tree)), cache(cache),
      joint_filter_params(joint_filter_params),
      contact_trigger_params(contact_trigger_params),
      kinematic_filter_params(kinematic_filter_params) {

  // initialize process noise covariance
  Q = Eigen::Matrix<double, 24, 24>::Identity();
  Q.topLeftCorner<3, 3>() *= kinematic_filter_params.Q_pos;
  Q.block<4, 4>(3, 3) *= kinematic_filter_params.Q_quat;
  Q.block<3, 3>(7, 7) *= kinematic_filter_params.Q_vel;
  for (int i = 0; i < 2; i++) {
    Q.block<3, 3>(10 + i * 7, 10 + i * 7) *= kinematic_filter_params.Q_foot_pos;
    Q.block<4, 4>(13 + i * 7, 13 + i * 7) *=
        kinematic_filter_params.Q_foot_quat;
  }
}

CassieState CassieKinematicFilter::get_full_state() { return x_cassie; }

std::vector<bool> CassieKinematicFilter::get_contact() { return contact; }

void CassieKinematicFilter::reset() { initialized = false; }

CassieJointVector CassieKinematicFilter::get_joint_positions() {
  CassieJointVector q;
  q << x_cassie.segment<16>(7);
  return q;
}

CassieJointVector CassieKinematicFilter::get_joint_velocities() {
  CassieJointVector v;
  v << x_cassie.segment<16>(29);
  return v;
}

void CassieKinematicFilter::set_joint_positions(CassieJointVector q) {
  x_cassie.segment<16>(7) = q;
}

void CassieKinematicFilter::set_joint_velocities(CassieJointVector v) {
  x_cassie.segment<16>(29) = v;
}

int CassieKinematicFilter::get_foot_body_idx(int foot_idx) {
  switch (foot_idx) {
  case 0:
    return tree->FindBodyIndex("toe_left");
  case 1:
    return tree->FindBodyIndex("toe_right");
  default:
    throw std::runtime_error("Foot index doesn't exist");
  }
}

double CassieKinematicFilter::get_spring_deflection(int foot_idx) {
  int spring_idx;
  switch (foot_idx) {
  case 0:
    spring_idx = 15;
    break;
  case 1:
    spring_idx = 16;
    break;
  default:
    throw std::runtime_error("Foot index doesn't exist");
  }
  return x_cassie[spring_idx];
}

void CassieKinematicFilter::get_foot_pose_in_base_frame(Eigen::Vector3d &p,
                                                        Eigen::Vector4d &q,
                                                        int foot_idx) {
  int base_idx = tree->FindBodyIndex("pelvis");
  int foot_body_idx = get_foot_body_idx(foot_idx);
  p = tree->transformPoints(cache, p_midfoot, foot_body_idx, base_idx);
  q = tree->relativeQuaternion(cache, foot_body_idx, base_idx);
}

void CassieKinematicFilter::joint_dynamics_update(
    double dt, const CassieJointVector &q_meas,
    const CassieJointVector &v_meas) {
  auto q_hat = get_joint_positions();
  auto v_hat = get_joint_velocities();

  // dynamics update
  q_hat = q_hat + dt * v_hat;

  // measurement update
  q_hat = joint_filter_params.alpha_q * q_meas +
          (1 - joint_filter_params.alpha_q) * q_hat;
  v_hat = joint_filter_params.alpha_v * v_meas +
          (1 - joint_filter_params.alpha_v) * v_hat;

  set_joint_positions(q_hat);
  set_joint_velocities(v_hat);
}

void CassieKinematicFilter::joint_constraint_update() {
  // TODO: implement this!
  Eigen::VectorXd phi = tree->positionConstraints(cache);
  Eigen::Matrix<double, Eigen::Dynamic, kCassiePositions> J_con =
      tree->positionConstraintsJacobian(cache, true);
  Eigen::Matrix<double, Eigen::Dynamic, kCassiePositions> J =
      Eigen::Matrix<double, 2, kCassiePositions>::Zero();
  J.col(19) = J_con.col(19);
  J.col(20) = J_con.col(20);
  x_cassie.head<kCassiePositions>() -=
      J.transpose() * (J * J.transpose()).inverse() * phi;
}

void CassieKinematicFilter::update_contact_mode_estimate() {
  // loop over feet
  for (int i = 0; i < 2; i++) {
    // Schmitt Trigger
    bool contact_last = contact[i];
    double limit = contact[i] ? contact_trigger_params.limit_upper
                              : contact_trigger_params.limit_lower;
    contact[i] = get_spring_deflection(i) < limit;

    // if the contact has become active reset the filter estimate for that foot
    if (!contact_last && contact[i]) {
      reset_foot(i);
    }
  }
}

void CassieKinematicFilter::reset_foot(int foot_idx) {
  int foot_body_idx = get_foot_body_idx(foot_idx);
  get_filter_foot_pos(x_filter, foot_idx) =
      tree->transformPoints(cache, p_midfoot, foot_body_idx, 0);
  get_filter_foot_quat(x_filter, foot_idx) =
      tree->relativeQuaternion(cache, foot_body_idx, 0);
}

void CassieKinematicFilter::update_filter_estimate() {
  std::vector<Vector<7>> y_vec;
  std::vector<Eigen::Matrix<double, 7, 7>> R_vec;
  for (int i = 0; i < 2; i++) {
    Eigen::Vector3d p;
    Eigen::Vector4d q;

    if (contact[i]) {
      get_foot_pose_in_base_frame(p, q, i);
    } else {
      p = Eigen::Vector3d::Zero();
      q = Eigen::Vector4d::Zero();
    }

    Vector<7> y;
    y << p, q;
    y_vec.push_back(y);

    Eigen::Matrix<double, 7, 7> R = Eigen::Matrix<double, 7, 7>::Identity();
    R.topLeftCorner<3, 3>() *= kinematic_filter_params.R_foot_pos;
    R.bottomRightCorner<4, 4>() *= kinematic_filter_params.R_foot_quat;
    R_vec.push_back(R);
  }

  kinematic_measurement_update(x_filter, P, contact, y_vec, R_vec);
}

void CassieKinematicFilter::initialize_state_estimate(
    const CassieMeasurement &meas) {

  // initialize covariance
  P = Eigen::Matrix<double, 24, 24>::Identity();
  P.topLeftCorner<3, 3>() *= kinematic_filter_params.P_pos;
  P.block<4, 4>(3, 3) *= kinematic_filter_params.P_quat;
  P.block<3, 3>(7, 7) *= kinematic_filter_params.P_vel;
  for (int i = 0; i < 2; i++) {
    P.block<3, 3>(10 + i * 7, 10 + i * 7) *= kinematic_filter_params.P_foot_pos;
    P.block<4, 4>(13 + i * 7, 13 + i * 7) *=
        kinematic_filter_params.P_foot_quat;
  }

  // populate the full state
  x_cassie.head<3>() = Eigen::Vector3d::Zero();
  x_cassie.segment<4>(3) = meas.orientation;
  set_joint_positions(meas.joint_positions);
  set_joint_velocities(meas.joint_velocities);
  x_cassie.segment<3>(23) = meas.gyro;
  x_cassie.segment<3>(26) = Eigen::Vector3d::Zero();

  // compute kinematics
  cache.initialize(x_cassie.head(kCassiePositions),
                   x_cassie.tail(kCassieVelocities));
  tree->doKinematics(cache);

  // find the lowest foot and translate that foot to the floor
  double min_pos = 1e6;
  for (int i = 0; i < 2; i++) {
    int foot_body_idx = get_foot_body_idx(i);
    auto p = get_filter_foot_pos(x_filter, i);
    auto q = get_filter_foot_quat(x_filter, i);
    p = tree->transformPoints(cache, p_midfoot, foot_body_idx, 0);
    q = tree->relativeQuaternion(cache, foot_body_idx, 0);

    min_pos = std::min(min_pos, p(2).value());
  }
  x_cassie(2) = -min_pos;

  // set the contacts to active
  for (int i = 0; i < 2; i++) {
    contact[i] = true;
  }

  // finish populating the filter state
  x_filter.head<7>() = x_cassie.head<7>();
  x_filter.segment<3>(7) = x_cassie.segment<3>(26);

  initialized = true;
}

void CassieKinematicFilter::update_quaternion_measurement(
    const Eigen::Vector4d &quat) {
  resetAutoDiff(x_filter);
  AutoDiffVector<4, 24> h = x_filter.segment<4>(3);
  auto H = autoDiffToGradientMatrix(h);
  Eigen::Matrix4d R_quat = R_imu_quat * Eigen::Matrix4d::Identity();

  ekf_measurement_update(x_filter, P, quat, h, H, R_quat);
}

void CassieKinematicFilter::update_height_measurement() {
  using Matrix1d = Eigen::Matrix<double, 1, 1>;
  Matrix1d y(0.0);
  resetAutoDiff(x_filter);
  for (int i = 0; i < 2; i++) {
    if (!contact[i]) {
      break;
    }
    AutoDiffVector<1, 24> h = get_filter_foot_pos(x_filter, i).tail<1>();
    auto H = autoDiffToGradientMatrix(h);
    Matrix1d R_height(R_foot_height);
    ekf_measurement_update(x_filter, P, y, h, H, R_height);
  }
}

void CassieKinematicFilter::update(double dt, CassieMeasurement meas) {
  if (!initialized) {
    initialize_state_estimate(meas);
  }

  // filter the joint signals
  joint_dynamics_update(dt, meas.joint_positions, meas.joint_velocities);

  // project onto the constraint plane
  // NOTE: this adds an expensive call to the kinematics and might not be
  // necessary
  cache.initialize(x_cassie.head<kCassiePositions>(),
                   x_cassie.tail<kCassieVelocities>());
  tree->doKinematics(cache);
  joint_constraint_update();

  // estimate the contact mode from springs
  update_contact_mode_estimate();

  // update the kinematic filter state
  dynamics_update(dt, x_filter, P, Q, meas.gyro, meas.accel);

  // do the kinematics
  x_cassie.head<7>() = autoDiffToValueMatrix(x_filter.head<7>());
  x_cassie.segment<3>(26) = autoDiffToValueMatrix(x_filter.segment<3>(7));
  x_cassie.segment<3>(23) = meas.gyro;
  cache.initialize(x_cassie.head<kCassiePositions>(),
                   x_cassie.tail<kCassieVelocities>());
  tree->doKinematics(cache);

  update_filter_estimate();
  // TODO: add measurements that the feet are on the ground
  update_quaternion_measurement(meas.orientation);
  update_height_measurement();

  x_cassie.head<7>() = autoDiffToValueMatrix(x_filter.head<7>());
  x_cassie.segment<3>(26) = autoDiffToValueMatrix(x_filter.segment<3>(7));
  x_cassie.segment<3>(23) = meas.gyro;
}

}  // namespace filter
}  // namespace cassie
