#include "cassie/cassie_common.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"

#include <memory>
#include <string>

namespace cassie {

template <typename Scalar>
std::unique_ptr<RigidBodyTree<Scalar>>
getCassieTree(CassieURDFType urdf_id, FloatingBaseType floating_base) {
  auto tree = std::make_unique<RigidBodyTree<Scalar>>();
  std::string urdf;
  double knee_stiffness = 0.0;
  double ankle_stiffness = 0.0;

  switch (urdf_id) {
  case kStandardCassie:
    urdf = "cassie/models/urdf/cassie.urdf";
    knee_stiffness = 1500;
    ankle_stiffness = 1250;
    break;
  case kSoftSpringsCassie:
    urdf = "cassie/models/urdf/cassie_soft_spring.urdf";
    knee_stiffness = 240;
    ankle_stiffness = 200;
    break;
  case kActiveSpringsCassie:
    urdf = "cassie/models/urdf/cassie_active_spring.urdf";
    break;
  case kActiveAnkleCassie:
    urdf = "cassie/models/urdf/cassie_active_ankle.urdf";
    break;
  case kFixedSpringCassie:
    urdf = "cassie/models/urdf/cassie_fixed_springs.urdf";
    break;
  }

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      urdf, floating_base, nullptr /* weld to frame */, tree.get());

  // add terrain
  if (floating_base != FloatingBaseType::kFixed) {
    drake::multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);
  }

  if (urdf_id != kActiveAnkleCassie) {
    const int thigh_left = tree->FindBodyIndex("thigh_left");
    const int thigh_right = tree->FindBodyIndex("thigh_right");
    const int heel_spring_left = tree->FindBodyIndex("heel_spring_left");
    const int heel_spring_right = tree->FindBodyIndex("heel_spring_right");

    tree->addDistanceConstraint(thigh_left, GetFourBarHipMountPoint(),
                                heel_spring_left, GetFourBarHeelMountPoint(),
                                kCassieFourBarDistance);
    tree->addDistanceConstraint(thigh_right, -GetFourBarHipMountPoint(),
                                heel_spring_right, GetFourBarHeelMountPoint(),
                                kCassieFourBarDistance);
  }

  if (urdf_id == kStandardCassie || urdf_id == kSoftSpringsCassie) {
    auto left_knee_spring = tree->FindChildBodyOfJoint("knee_joint_left");
    auto right_knee_spring = tree->FindChildBodyOfJoint("knee_joint_right");
    auto left_ankle_spring =
        tree->FindChildBodyOfJoint("ankle_spring_joint_left");
    auto right_ankle_spring =
        tree->FindChildBodyOfJoint("ankle_spring_joint_right");

    RevoluteJoint& left_knee_spring_joint = dynamic_cast<RevoluteJoint&>(
        left_knee_spring->get_mutable_joint());
    RevoluteJoint& right_knee_spring_joint = dynamic_cast<RevoluteJoint&>(
        right_knee_spring->get_mutable_joint());
    RevoluteJoint& left_ankle_spring_joint = dynamic_cast<RevoluteJoint&>(
        left_ankle_spring->get_mutable_joint());
    RevoluteJoint& right_ankle_spring_joint = dynamic_cast<RevoluteJoint&>(
        right_ankle_spring->get_mutable_joint());

    left_knee_spring_joint.SetSpringDynamics(knee_stiffness, 0.0);
    right_knee_spring_joint.SetSpringDynamics(knee_stiffness, 0.0);
    left_ankle_spring_joint.SetSpringDynamics(ankle_stiffness, 0.0);
    right_ankle_spring_joint.SetSpringDynamics(ankle_stiffness, 0.0);
  }

  return tree;
}

std::unique_ptr<RigidBodyTree<double>>
getCassieTreed(CassieURDFType urdf_id, FloatingBaseType floating_base) {
  return getCassieTree<double>(urdf_id, floating_base);
}

template <typename T>
void setDefaultContactParams(RigidBodyPlant<T>& plant) {
  const double kYoungsModulus = 1e8;  // Pa
  const double kDissipation = 5.0;  // s/m
  const double kStaticFriction = 0.9;
  const double kDynamicFriction = 0.5;
  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(kYoungsModulus)
      .set_dissipation(kDissipation)
      .set_friction(kStaticFriction, kDynamicFriction);
  plant.set_default_compliant_material(default_material);

  const double kStictionSlipTolerance = 0.01;  // m/s
  const double kContactRadius = 2e-3;  // m
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = kContactRadius;
  model_parameters.v_stiction_tolerance = kStictionSlipTolerance;
  plant.set_contact_model_parameters(model_parameters);
}

template void setDefaultContactParams<double>(RigidBodyPlant<double>&);

VectorX<double> CassieFixedPointState(FloatingBaseType floating_base) {
  VectorX<double> ret(kCassieStates);
  ret << 0, 0, 0.9075, 1, 0, 0, 0, 0, 0, 0.0029, -0.0029, 0.6322, 0.6322,
          -1.4376, -1.4376, -0.0416, -0.0416, 1.7466, 1.7466, -0.0325, -0.0325,
          -1.7731, -1.7731, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  switch (floating_base) {
  case FloatingBaseType::kQuaternion:
    return ret;
    break;
  case FloatingBaseType::kRollPitchYaw: {
    VectorX<double> ret_rpy(kCassieStates - 1);
    ret_rpy << ret.head(3), ret.tail(kCassieStates - 4);
    return ret_rpy;
  } break;
  case FloatingBaseType::kFixed:
    return ret.segment(7, kCassieStates - 13);
    break;
  default:
    throw std::invalid_argument(
        "Only quaternion, rpy and fixed floating base supported.");
  }
}


VectorX<double> CassieFixedPointTorque() {
  VectorX<double> ff_torque(kCassieActuators);
  ff_torque << 2.5262, -2.5356, 0, 0, -5.9186, -6.0391, 47.6785, 47.6178,
      0.8013, 0.6745;
  return ff_torque;
}

Vector3<double> GetFourBarHipMountPoint() {
  Vector3<double> hip_mount_point(0, 0, 0.045);
  return hip_mount_point;
}

Vector3<double> GetFourBarHeelMountPoint() {
  Vector3<double> heel_mount_point(0.11877, -0.01, 0);
  return heel_mount_point;
}

FloatingBaseState BaseStateFromFullState(const CassieState &full_state) {
  FloatingBaseState base_state;
  base_state << full_state.segment(0, 7), full_state.segment(23, 6);
  return base_state;
}


}  // namespace cassie
