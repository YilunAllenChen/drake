#pragma once

#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"

#include <memory>

namespace cassie {
using drake::systems::RigidBodyPlant;
using drake::multibody::joints::FloatingBaseType;

constexpr int kCassiePositions = 23;
constexpr int kCassieVelocities = 22;
constexpr int kCassieStates = 22+23;
constexpr int kCassieJoints = 16;
constexpr int kCassieActuators = 10;
constexpr double kCassieFourBarDistance = 0.5012;
constexpr int kCassieFilterSize = 10+2*7;


const Eigen::Vector3d p_midfoot(0.0211, 0.0560, 0);


typedef Eigen::Matrix<double, kCassiePositions, 1> CassiePosition;
typedef Eigen::Matrix<double, kCassieVelocities, 1> CassieVelocity;
typedef Eigen::Matrix<double, kCassieStates, 1> CassieState;
typedef Eigen::Matrix<double, kCassieJoints, 1> CassieJointVector;

typedef Eigen::Matrix<double, 13, 1> FloatingBaseState;
typedef Eigen::Matrix<double, 13, 13> FloatingBaseCov;
typedef Eigen::AutoDiffScalar<CassieState> AutoDiffCassieState;

enum CassieURDFType {
  kStandardCassie = 0,
  kSoftSpringsCassie = 1,
  kActiveSpringsCassie = 2,
  kActiveAnkleCassie = 3,
  kFixedSpringCassie = 4
};

template<typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

template<typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

template<typename Scalar>
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

VectorX<double> CassieFixedPointState(
    FloatingBaseType floating_base = FloatingBaseType::kQuaternion);

VectorX<double> CassieFixedPointTorque();

Vector3<double> GetFourBarHipMountPoint();

Vector3<double> GetFourBarHeelMountPoint();

FloatingBaseState BaseStateFromFullState(const CassieState&);

std::unique_ptr<RigidBodyTree<double>>
getCassieTreed(CassieURDFType urdf_id = kStandardCassie,
               FloatingBaseType floating_base = FloatingBaseType::kQuaternion);

template <typename Scalar>
std::unique_ptr<RigidBodyTree<Scalar>>
getCassieTree(CassieURDFType urdf_id = kStandardCassie,
              FloatingBaseType floating_base = FloatingBaseType::kQuaternion);

template <typename T>
void setDefaultContactParams(RigidBodyPlant<T>& plant);

}  // namespace cassie
