#include "cassie/cassie_common.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

#include <gtest/gtest.h>

namespace cassie {
namespace {
using drake::AutoDiffXd;
using drake::MatrixCompareType;
using drake::multibody::joints::FloatingBaseType;
using Eigen::Vector3d;
using Eigen::VectorXd;

GTEST_TEST(CassieCommon, CheckFixedValues) {
  EXPECT_EQ(kCassiePositions, 23);
  EXPECT_EQ(kCassieVelocities, 22);
  EXPECT_EQ(kCassieStates, 22 + 23);
  EXPECT_EQ(kCassieActuators, 10);
  EXPECT_EQ(kCassieFourBarDistance, 0.5012);

  Vector3d p_midfoot_real(0.0211, 0.0560, 0);
  EXPECT_TRUE(CompareMatrices(p_midfoot, p_midfoot_real, 1e-8,
                              MatrixCompareType::relative));

  VectorXd fixed_pt = CassieFixedPointState();
  VectorXd fixed_pt_real(kCassieStates);
  fixed_pt_real << 0, 0, 0.9075, 1, 0, 0, 0, 0, 0, 0.0029, -0.0029,
      0.6322, 0.6322, -1.4376, -1.4376, -0.0416, -0.0416, 1.7466, 1.7466,
      -0.0325, -0.0325, -1.7731, -1.7731, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  VectorXd fixed_pt_real_rpy(kCassieStates - 1);
  fixed_pt_real_rpy << fixed_pt_real.head(3),
      fixed_pt_real.tail(kCassieStates - 4);
  VectorX<double> fixed_pt_real_fixed =
      fixed_pt_real.segment(7, kCassieStates - 13);
  EXPECT_TRUE(CompareMatrices(fixed_pt, fixed_pt_real, 1e-8,
                              MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(CassieFixedPointState(FloatingBaseType::kRollPitchYaw),
                      fixed_pt_real_rpy, 1e-8, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(CassieFixedPointState(FloatingBaseType::kFixed),
                              fixed_pt_real_fixed, 1e-8,
                              MatrixCompareType::relative));
  EXPECT_THROW(
      CassieFixedPointState(FloatingBaseType::kExperimentalMultibodyPlantStyle),
      std::invalid_argument);

  VectorXd fixed_u = CassieFixedPointTorque();
  VectorX<double> fixed_u_real(kCassieActuators);
  fixed_u_real << 2.5262, -2.5356, 0, 0, -5.9186, -6.0391, 47.6785, 47.6178,
      0.8013, 0.6745;
  EXPECT_TRUE(CompareMatrices(fixed_u, fixed_u_real, 1e-8,
                              MatrixCompareType::relative));

  Vector3d hip_pt = GetFourBarHipMountPoint();
  Vector3d hip_pt_real(0, 0, 0.045);
  EXPECT_TRUE(
      CompareMatrices(hip_pt, hip_pt_real, 1e-8, MatrixCompareType::relative));

  Vector3d heel_pt = GetFourBarHeelMountPoint();
  Vector3d heel_pt_real(0.11877, -0.01, 0);
  EXPECT_TRUE(CompareMatrices(heel_pt, heel_pt_real, 1e-8,
                              MatrixCompareType::relative));
}

template <typename Scalar>
void checkCassieTree(const std::unique_ptr<RigidBodyTree<Scalar>> &rtree,
                     int nq, int nv, int nu, CassieURDFType urdf_id) {
  EXPECT_EQ(rtree->get_num_positions(), nq);
  EXPECT_EQ(rtree->get_num_velocities(), nv);
  EXPECT_EQ(rtree->get_num_actuators(), nu);
  if (urdf_id != kActiveAnkleCassie) {
    EXPECT_EQ(rtree->getNumPositionConstraints(), 2);
  } else {
    EXPECT_EQ(rtree->getNumPositionConstraints(), 0);
  }

  if (urdf_id == kStandardCassie || urdf_id == kSoftSpringsCassie) {
    VectorXd q = VectorXd::Zero(nq);
    q(nq - 8) = 0.1;
    q(nq - 7) = 0.1;
    q(nq - 4) = 0.1;
    q(nq - 3) = 0.1;
    VectorXd spring = rtree->CalcGeneralizedSpringForces(q);
    VectorXd spring_real = VectorXd::Zero(nv);
    if (urdf_id == kStandardCassie) {
      spring_real(nv - 8) = -150;
      spring_real(nv - 7) = -150;
      spring_real(nv - 4) = -125;
      spring_real(nv - 3) = -125;
    } else {
      spring_real(nv - 8) = -24;
      spring_real(nv - 7) = -24;
      spring_real(nv - 4) = -20;
      spring_real(nv - 3) = -20;
    }
    EXPECT_TRUE(CompareMatrices(spring, spring_real, 1e-8,
                                MatrixCompareType::relative));
  }
}

GTEST_TEST(CassieCommon, GetCassieTree) {
  // Check different floating bases
  auto rtree = getCassieTreed();
  checkCassieTree(rtree, 23, 22, 10, kStandardCassie);
  rtree = getCassieTreed(kStandardCassie, FloatingBaseType::kRollPitchYaw);
  checkCassieTree(rtree, 22, 22, 10, kStandardCassie);
  rtree = getCassieTreed(kStandardCassie, FloatingBaseType::kFixed);
  checkCassieTree(rtree, 16, 16, 10, kStandardCassie);

  // Check different urdfs
  rtree = getCassieTreed(kSoftSpringsCassie);
  checkCassieTree(rtree, 23, 22, 10, kSoftSpringsCassie);
  rtree = getCassieTreed(kActiveSpringsCassie);
  checkCassieTree(rtree, 23, 22, 14, kActiveSpringsCassie);
  rtree = getCassieTreed(kActiveAnkleCassie);
  checkCassieTree(rtree, 19, 18, 12, kActiveAnkleCassie);
  rtree = getCassieTreed(kFixedSpringCassie);
  checkCassieTree(rtree, 19, 18, 10, kFixedSpringCassie);

  // Check different scalar types
  auto rtree_d = getCassieTree<double>();
  checkCassieTree(rtree_d, 23, 22, 10, kStandardCassie);
  // TODO: add test for autodiff/other scalars
}

GTEST_TEST(CassieCommon, HelperFunctions) {
  auto tree = getCassieTreed();
  RigidBodyPlant<double> plant(std::move(tree), 0.0005);
  setDefaultContactParams(plant);

  CassieState x = CassieFixedPointState();
  FloatingBaseState pelvis = BaseStateFromFullState(x);
  VectorXd pelvis_real(13);
  pelvis_real << 0, 0, 0.9075, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  EXPECT_TRUE(
      CompareMatrices(pelvis, pelvis_real, 1e-8, MatrixCompareType::relative));
}

}  // namespace
}  // namespace cassie
