#include "cassie/cassie_common.h"
#include "cassie/filter/kinematic_filter.h"
#include "drake/multibody/rigid_body_tree.h"

#include <memory>
#include <vector>

namespace cassie {
namespace filter {

typedef Eigen::Matrix<double, 16, 1> MeasuredJointVector;

struct CassieMeasurement {
  MeasuredJointVector joint_positions;
  MeasuredJointVector joint_velocities;
  Eigen::Vector4d orientation;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
};

struct JointFilterParams {
  double alpha_q = 0.6;
  double alpha_v = 1.0;
};

struct ContactTriggerParams {
  double limit_upper = -0.01;
  double limit_lower = -0.02;
};

struct KinematicFilterParams {
  double P_pos = 1e-2;
  double P_quat = 1e-2;
  double P_vel = 1e-2;
  double P_foot_pos = 1e-2;
  double P_foot_quat = 1e-2;
  double Q_pos = 1e-3;
  double Q_quat = 1e-3;
  double Q_vel = 1e-3;
  double Q_foot_pos = 1e-3;
  double Q_foot_quat = 1e-3;
  double R_foot_pos = 1e-6;
  double R_foot_quat = 1e-6;
};

class CassieKinematicFilter {
 public:
  CassieKinematicFilter(std::unique_ptr<RigidBodyTree<double>> tree,
                        KinematicsCache<double> cache,
                        JointFilterParams joint_filter_params,
                        ContactTriggerParams contact_trigger_params,
                        KinematicFilterParams kinematic_filter_params);

  CassieState get_full_state();
  std::vector<bool> get_contact();
  void reset();
  void update(double dt, CassieMeasurement meas);

 private:
  bool initialized = false;
  std::unique_ptr<RigidBodyTree<double>> tree;
  KinematicsCache<double> cache;
  JointFilterParams joint_filter_params;
  ContactTriggerParams contact_trigger_params;
  KinematicFilterParams kinematic_filter_params;
  std::vector<bool> contact = {true, true};
  Eigen::Matrix<double, kCassieStates, 1> x_cassie;
  AutoDiffVector<24, 24> x_filter;
  Eigen::Matrix<double, 24, 24> P;
  Eigen::Matrix<double, 24, 24> Q;
  double R_imu_quat = 1e-4;
  double R_foot_height = 1e-4;

  /* helper functions */
  CassieJointVector get_joint_positions();
  CassieJointVector get_joint_velocities();
  void set_joint_positions(CassieJointVector q);
  void set_joint_velocities(CassieJointVector v);
  int get_foot_body_idx(int foot_idx);
  double get_spring_deflection(int foot_idx);
  void get_foot_pose_in_base_frame(Eigen::Vector3d &p, Eigen::Vector4d &q,
                                   int foot_idx);

  /*
  Update the joint angles
  */
  void joint_dynamics_update(double dt, const CassieJointVector &q_meas,
                             const CassieJointVector &v_meas);
  /*
  Enforce any constraints
  */
  void joint_constraint_update();

  /*
  Update the estimated contact mode using a Schmitt Trigger on the spring
  deflections
  */
  void update_contact_mode_estimate();

  /*
  Resets the foot position according to the kinematics contact becomes active
  again
  */
  void reset_foot(int foot_idx);

  /*
  Update the floating base state
  */
  void update_filter_estimate();

  /*
  Uses only the measurement data to update the state estimate
  */
  void initialize_state_estimate(const CassieMeasurement &meas);

  /*
  Additional measurement updates
  */
  void update_quaternion_measurement(const Eigen::Vector4d &quat);
  void update_height_measurement();
};

}  // namespace filter
}  // namespace cassie
