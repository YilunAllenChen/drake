#include "cassie/cassie_common.h"
#include "cassie/control/control_util.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

#include <gtest/gtest.h>
#include <math.h>

namespace cassie {
namespace control {
namespace {
using drake::MatrixCompareType;
using Eigen::Matrix;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

GTEST_TEST(ControlUtil, ContactPositionAndJacobian) {
  auto cassie = getCassieTreed();

  VectorXd x(kCassieStates);
  x << 0, 0, 0.9342, 1, 0, 0, 0, 0, 0, 0.0057, 0.0057, 0.6726, 0.6726,
        -1.4100, -1.4100, -0.0374, -0.0374, 1.6493, 1.6493, -0.0289, -0.0289,
        -1.7479, -1.7479, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;
  VectorXd q = x.head(kCassiePositions);
  VectorXd v = VectorXd::Constant(kCassieVelocities, 1);

  KinematicsCache<double> kinsol = cassie->doKinematics(q, v);
  RigidBody<double> *lfoot = cassie->FindBody("toe_left");
  RigidBody<double> *rfoot = cassie->FindBody("toe_right");

  Matrix3Xd pts = forwardKinTerrainPoints(*cassie, kinsol, *lfoot, *rfoot);
  MatrixXd J_qd = contactJacobian(*cassie, kinsol, *lfoot, *rfoot, true);
  MatrixXd J_v = contactJacobian(*cassie, kinsol, *lfoot, *rfoot, false);
  VectorXd Jdot_v = contactJacobianDotTimesV(*cassie, kinsol, *lfoot, *rfoot);

  Matrix3Xd pts_real(3, 4);
  pts_real << 1.24555806e-01, -4.98537730e-02, 1.24504504e-01, -4.99050756e-02,
              1.31489262e-01, 1.30495060e-01, -1.29510884e-01, -1.30505086e-01,
              1.57213798e-05, 1.96935403e-07, 1.57213798e-05, 1.96935402e-07;
  MatrixXd J_qd_real(12, kCassiePositions);
  J_qd_real << 1, 0, 0, 0, 0, -1.86836856, -2.62978525e-01, 1.14729113e-09,
                0, 3.51073758e-03, 0, 8.44170540e-01, 0, 7.50307555e-01, 0,
                6.73515446e-01, 0, 3.48866638e-01, 0, 0, 0, 5.64993010e-02, 0,
               0, 1, 0, 0, 1.86836856, 0, 2.49111612e-01, 9.34184312e-01,
                0, 1.73555806e-01, 0, 4.81210007e-03, 0, 4.27704459e-03, 0,
                3.83929973e-03, 0, 1.98867538e-03, 0, 0, 0, 3.22067968e-04, 0,
               0, 0, 1, 0, 2.62978525e-01, -2.49111612e-01, 0, -3.51073758e-03,
                0, 0, 0, 1.73532974e-01, 0, 9.87703249e-02, 0, 1.04475653e-01,
                0, 3.94330762e-01, 0, 0, 0, 1.06958942e-01, 0,
               1, 0, 0, 0, 0, -1.86839961, -2.60990120e-01, 1.47219138e-09,
                0, 4.50493995e-03, 0, 8.44186065e-01, 0, 7.50323079e-01, 0,
                6.73530970e-01, 0, 3.48882162e-01, 0, 0, 0, 5.65148252e-02, 0,
               0, 1, 0, 0, 1.86839961, 0, -9.97075459e-02, 9.34199780e-01,
                0, -8.53772971e-04, 0, 4.81218857e-03, 0, 4.27713308e-03, 0,
                3.83938822e-03, 0, 1.98876388e-03, 0, 0, 0, 3.22156462e-04, 0,
               0, 0, 1, 0, 2.60990120e-01, 9.97075459e-02, 0, -4.50493995e-03,
                0, 0, 0, -8.79438591e-04, 0, -7.56420879e-02, 0,
                -6.99367601e-02, 0, 2.19918349e-01, 0, 0, 0, -6.74534710e-02, 0,
               1, 0, 0, 0, 0, -1.86836856, 2.59021768e-01, 0, -1.79381516e-09,
                0, -5.48911620e-03, 0, 8.44170540e-01, 0, 7.50307555e-01, 0,
                6.73515446e-01, 0, 3.48866638e-01, 0, 0, 0, 5.64993010e-02,
               0, 1, 0, 0, 1.86836856, 0, 2.49009007e-01, 0, 9.34184312e-01,
                0, 1.73504504e-01, 0, 4.81210007e-03, 0, 4.27704459e-03, 0,
                3.83929973e-03, 0, 1.98867538e-03, 0, 0, 0, 3.22067968e-04,
               0, 0, 1, 0, -2.59021768e-01, -2.49009007e-01, 0, 0,
                5.48911620e-03, 0, 0, 0, 1.73532974e-01, 0, 9.87703249e-02, 0,
                1.04475653e-01, 0, 3.94330762e-01, 0, 0, 0, 1.06958942e-01,
               1, 0, 0, 0, 0, -1.86839961, 2.61010172e-01, 0, -1.46891490e-09,
                0, -4.49491383e-03, 0, 8.44186065e-01, 0, 7.50323079e-01, 0,
                6.73530970e-01, 0, 3.48882162e-01, 0, 0, 0, 5.65148252e-02,
               0, 1, 0, 0, 1.86839961, 0, -9.98101513e-02, 0, 9.34199780e-01,
                0, -9.05075635e-04, 0, 4.81218857e-03, 0, 4.27713308e-03, 0,
                3.83938822e-03, 0, 1.98876388e-03, 0, 0, 0, 3.22156462e-04,
               0, 0, 1, 0, -2.61010172e-01, 9.98101513e-02, 0, 0,
                4.49491383e-03, 0, 0, 0, -8.79438591e-04, 0, -7.56420879e-02, 0,
                -6.99367601e-02, 0, 2.19918349e-01, 0, 0, 0, -6.74534710e-02;
  MatrixXd J_v_real(12, kCassieVelocities);
  J_v_real << 0, -9.34184279e-01, -1.31489262e-01, 1, 0, 0, 1.14729113e-09, 0,
                3.51073758e-03, 0, 8.44170540e-01, 0, 7.50307555e-01, 0,
                6.73515446e-01, 0, 3.48866638e-01, 0, 0, 0, 5.64993010e-02, 0,
              9.34184279e-01, 0, 1.24555806e-01, 0, 1, 0, 9.34184312e-01, 0,
                1.73555806e-01, 0, 4.81210007e-03, 0, 4.27704459e-03, 0,
                3.83929973e-03, 0, 1.98867538e-03, 0, 0, 0, 3.22067968e-04, 0,
              1.31489262e-01, -1.24555806e-01, 0, 0, 0, 1, -3.51073758e-03, 0,
                0, 0, 1.73532974e-01, 0, 9.87703249e-02, 0, 1.04475653e-01, 0,
                3.94330762e-01, 0, 0, 0, 1.06958942e-01, 0,
              0, -9.34199803e-01, -1.30495060e-01, 1, 0, 0, 1.47219138e-09, 0,
                4.50493995e-03, 0, 8.44186065e-01, 0, 7.50323079e-01, 0,
                6.73530970e-01, 0, 3.48882162e-01, 0, 0, 0, 5.65148252e-02, 0,
              9.34199803e-01, 0, -4.98537730e-02, 0, 1, 0, 9.34199780e-01, 0,
                -8.53772971e-04, 0, 4.81218857e-03, 0, 4.27713308e-03, 0,
                3.83938822e-03, 0, 1.98876388e-03, 0, 0, 0, 3.22156462e-04, 0,
              1.30495060e-01, 4.98537730e-02, 0, 0, 0, 1, -4.50493995e-03, 0,
                0, 0, -8.79438591e-04, 0, -7.56420879e-02, 0, -6.99367601e-02,
                0, 2.19918349e-01, 0, 0, 0, -6.74534710e-02, 0,
              0, -9.34184279e-01, 1.29510884e-01, 1, 0, 0, 0, -1.79381516e-09,
                0, -5.48911620e-03, 0,  8.44170540e-01, 0, 7.50307555e-01, 0,
                6.73515446e-01, 0, 3.48866638e-01, 0, 0, 0, 5.64993010e-02,
              9.34184279e-01, 0, 1.24504504e-01, 0, 1, 0, 0, 9.34184312e-01,
                0, 1.73504504e-01, 0, 4.81210007e-03, 0, 4.27704459e-03, 0,
                3.83929973e-03, 0, 1.98867538e-03, 0, 0, 0, 3.22067968e-04,
             -1.29510884e-01, -1.24504504e-01, 0, 0, 0, 1, 0, 5.48911620e-03,
                0, 0, 0, 1.73532974e-01, 0, 9.87703249e-02, 0, 1.04475653e-01,
                0, 3.94330762e-01, 0, 0, 0, 1.06958942e-01,
              0, -9.34199803e-01, 1.30505086e-01, 1, 0, 0, 0, -1.46891490e-09,
                0, -4.49491383e-03, 0, 8.44186065e-01, 0, 7.50323079e-01, 0,
                6.73530970e-01, 0, 3.48882162e-01, 0, 0, 0, 5.65148252e-02,
              9.34199803e-01, 0, -4.99050756e-02, 0, 1, 0, 0, 9.34199780e-01,
                0, -9.05075635e-04, 0, 4.81218857e-03, 0, 4.27713308e-03, 0,
                3.83938822e-03, 0, 1.98876388e-03, 0, 0, 0, 3.22156462e-04,
             -1.30505086e-01, 4.99050756e-02, 0, 0, 0, 1, 0, 4.49491383e-03,
                0, 0, 0, -8.79438591e-04, 0, -7.56420879e-02, 0,
                -6.99367601e-02, 0, 2.19918349e-01, 0, 0, 0, -6.74534710e-02;
  VectorXd Jdot_v_real(12);
  Jdot_v_real << -6.46777543, 6.10274462, 9.74178965, -2.98264291, 9.44968694,
                 8.86707123, -6.71051936, 6.57069448, 9.4985327, -3.22538683,
                 9.9176368, 8.62381429;

  EXPECT_TRUE(
      CompareMatrices(pts, pts_real, 1e-8, MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(J_qd, J_qd_real, 1e-8, MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(J_v, J_v_real, 1e-8, MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(Jdot_v, Jdot_v_real, 1e-8, MatrixCompareType::relative));
}

GTEST_TEST(ControlUtil, SurfaceTangents) {
  Matrix<double, 3, m_surface_tangents> tangent;
  Matrix<double, 3, m_surface_tangents> tangent1;
  tangent1 << 1, 0, -1, 0,
              0, -1, 0, 1,
              0, 0, 0, 0;
  Matrix<double, 3, m_surface_tangents> tangent2;
  tangent2 << -1, 0, 1, 0,
              0, -1, 0, 1,
              0, 0, 0, 0;
  Matrix<double, 3, m_surface_tangents> tangent3;
  tangent3 << 1 / sqrt(2), 0, -1 / sqrt(2), 0,
              -1 / sqrt(2), 0, 1 / sqrt(2), 0,
              0, 1, 0, -1;

  Vector3d normal1;
  normal1 << 0, 0, 1;
  tangent = surfaceTangents(normal1);
  EXPECT_TRUE(
      CompareMatrices(tangent, tangent1, 1e-12, MatrixCompareType::relative));

  Vector3d normal2;
  normal2 << 0, 0, -1;
  tangent = surfaceTangents(normal2);
  EXPECT_TRUE(
      CompareMatrices(tangent, tangent2, 1e-12, MatrixCompareType::relative));

  Vector3d normal3;
  normal3 << 1, 1, 0;
  normal3.normalize();
  tangent = surfaceTangents(normal3);
  EXPECT_TRUE(
      CompareMatrices(tangent, tangent3, 1e-12, MatrixCompareType::relative));
}

GTEST_TEST(ControlUtil, ContactDistanceAndForces) {
  auto cassie = getCassieTreed();

  VectorXd x(kCassieStates);
  x << 0, 0, 0.9342, 1, 0, 0, 0, 0, 0, 0.0057, 0.0057, 0.6726, 0.6726,
        -1.4100, -1.4100, -0.0374, -0.0374, 1.6493, 1.6493, -0.0289, -0.0289,
        -1.7479, -1.7479, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;;
  VectorXd q = x.head(kCassiePositions);
  VectorXd v = VectorXd::Constant(kCassieVelocities, 1);

  KinematicsCache<double> kinsol = cassie->doKinematics(q, v);
  RigidBody<double> *lfoot = cassie->FindBody("toe_left");
  RigidBody<double> *rfoot = cassie->FindBody("toe_right");

  VectorXd phi;
  Matrix3Xd normals;
  contactDistancesAndNormals(*cassie, kinsol, *lfoot, *rfoot, phi, normals);

  VectorXd phi_real(4);
  phi_real << 1.57213798e-05, 1.96935402e-07, 1.57213798e-05, 1.96935403e-07;
  Matrix3Xd normals_real(3, 4);
  normals_real << 0, 0, 0, 0,
                  0, 0, 0, 0,
                  1, 1, 1, 1;
  EXPECT_TRUE(
      CompareMatrices(phi, phi_real, 1e-8, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(normals, normals_real, 1e-8,
                              MatrixCompareType::relative));

  Matrix3Xd B_q;
  MatrixXd JB_q;
  contactJacobianBV(*cassie, kinsol, *lfoot, *rfoot, true, B_q, JB_q);

  Matrix3Xd B_v;
  MatrixXd JB_v;
  contactJacobianBV(*cassie, kinsol, *lfoot, *rfoot, false, B_v, JB_v);

  Matrix3Xd B_real(3, 16);
  B_real << 7.07106781e-01, 0, -7.07106781e-01, 0,
                7.07106781e-01, 0, -7.07106781e-01, 0,
                7.07106781e-01, 0, -7.07106781e-01, 0,
                7.07106781e-01, 0, -7.07106781e-01, 0,
              0, -7.07106781e-01, 0, 7.07106781e-01,
                0, -7.07106781e-01, 0, 7.07106781e-01,
                0, -7.07106781e-01, 0, 7.07106781e-01,
                0, -7.07106781e-01, 0, 7.07106781e-01,
              7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
                7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
                7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
                7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01;
  MatrixXd JB_q_real(kCassiePositions, 16);
  JB_q_real << 7.07106781e-01, 0, -7.07106781e-01, 0, 7.07106781e-01, 0,
                -7.07106781e-01, 0, 7.07106781e-01, 0, -7.07106781e-01, 0,
                 7.07106781e-01, 0, -7.07106781e-01, 0,
             0, -7.07106781e-01, 0, 7.07106781e-01, 0, -7.07106781e-01, 0,
                7.07106781e-01, 0, -7.07106781e-01, 0, 7.07106781e-01, 0,
                -7.07106781e-01, 0, 7.07106781e-01,
             7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
                7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
                7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
                7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             1.85953898e-01, -1.13518218, 1.85953898e-01, 1.50708997,
                1.84547884e-01, -1.13661015, 1.84547884e-01, 1.50570592,
                -1.83156048e-01, -1.50429212, -1.83156048e-01, 1.13798003,
                -1.84562063e-01, -1.50572009, -1.84562063e-01, 1.13659597,
             -1.49728459, -1.76148510e-01, 1.14498757, -1.76148510e-01,
                -1.25065415, 7.05038819e-02, 1.39166191, 7.05038819e-02,
                -1.49721203, -1.76075958e-01, 1.14506012, -1.76075958e-01,
                -1.25058160, 7.05764348e-02, 1.39173447, 7.05764348e-02,
             -1.85953898e-01, -1.76148510e-01, 1.85953898e-01, 1.76148510e-01,
                -1.84547884e-01, 7.05038819e-02, 1.84547884e-01, -7.0503882e-02,
                1.83156048e-01, -1.76075958e-01, -1.83156048e-01, 1.7607596e-01,
                1.84562063e-01, 7.05764348e-02, -1.84562063e-01, -7.0576435e-02,
             -2.48246554e-03, -6.63050529e-01, -2.48246716e-03, 6.58085596e-01,
                -3.18547255e-03, -6.63764473e-01, -3.18547463e-03,
                6.57393526e-01, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 3.88139002e-03, -6.56686671e-01,
                3.88139255e-03, 6.64449453e-01, 3.17838301e-03,
                -6.57400615e-01, 3.17838509e-03, 6.63757383e-01,
             2.48246635e-03, -1.22722488e-01, -2.48246635e-03, 1.22722488e-01,
                3.18547359e-03, 6.03708658e-04, -3.18547359e-03,
                -6.03708658e-04, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, -3.88139128e-03, -1.22686211e-01,
                3.88139128e-03, 1.22686211e-01, -3.17838405e-03,
                6.39985119e-04, 3.17838405e-03, -6.39985119e-04,
             7.19625056e-01, 1.19303674e-01, -4.74212371e-01, 1.26109011e-01,
                5.96307834e-01, -4.02458816e-03, -5.97551548e-01,
                2.78087418e-03, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 7.19625056e-01, 1.19303674e-01,
                -4.74212371e-01, 1.26109011e-01, 5.96307834e-01,
                -4.02458816e-03, -5.97551548e-01, 2.78087418e-03,
             6.00388727e-01, 6.68168393e-02, -4.60706394e-01, 7.28654938e-02,
                4.77071504e-01, -5.65114231e-02, -5.84045571e-01,
                -5.04626435e-02, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 6.00388727e-01, 6.68168393e-02,
                -4.60706394e-01, 7.28654938e-02, 4.77071504e-01,
                -5.65114231e-02, -5.84045571e-01, -5.04626435e-02,
             5.50122781e-01, 7.11606476e-02, -4.02371896e-01, 7.65902374e-02,
                4.26805559e-01, -5.21676148e-02, -5.25711073e-01,
                -4.67378999e-02, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 5.50122781e-01, 7.11606476e-02,
                -4.02371896e-01, 7.65902374e-02, 4.26805559e-01,
                -5.21676148e-02, -5.25711073e-01, -4.67378999e-02,
             5.25519921e-01, 2.77427750e-01, 3.21479905e-02, 2.80240162e-01,
                4.02202698e-01, 1.54099487e-01, -9.11911866e-02,
                1.56912024e-01, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 5.25519921e-01, 2.77427750e-01,
                3.21479905e-02, 2.80240162e-01, 4.02202698e-01,
                1.54099487e-01, -9.11911866e-02, 1.56912024e-01,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             1.15582432e-01, 7.54036567e-02, 3.56803542e-02, 7.58591296e-02,
                -7.73479056e-03, -4.79246057e-02, -8.76588229e-02,
                -4.74690077e-02, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 1.15582432e-01, 7.54036567e-02,
                3.56803542e-02, 7.58591296e-02, -7.73479056e-03,
                -4.79246057e-02, -8.76588229e-02, -4.74690077e-02;
  MatrixXd JB_v_real(kCassieVelocities, 16);
  JB_v_real << 0.092976949, -0.567591089, 0.092976949, 0.753544987,
                0.092273942, -0.568305074, 0.092273942, 0.752852958,
                -0.091578024, -0.752146062, -0.091578024, 0.568990014,
                -0.092281031, -0.752860047, -0.092281031, 0.568297984,
             -0.748642294, -0.088074255, 0.572493783, -0.088074255,
                -0.625327075, 0.035251941, 0.695830957, 0.035251941,
                -0.748606017, -0.088037979, 0.572530060, -0.088037979,
                -0.625290798, 0.035288217, 0.695867233, 0.035288217,
             -0.092976949, -0.088074255, 0.092976949, 0.088074255,
                -0.092273942, 0.035251941, 0.092273942, -0.035251941,
                0.091578024, -0.088037979, -0.091578024, 0.088037979,
                0.092281031, 0.035288217, -0.092281031, -0.035288217,
             7.07106781e-01, 0, -7.07106781e-01, 0, 7.07106781e-01, 0,
                -7.07106781e-01, 0, 7.07106781e-01, 0, -7.07106781e-01, 0,
                 7.07106781e-01, 0, -7.07106781e-01, 0,
             0, -7.07106781e-01, 0, 7.07106781e-01, 0, -7.07106781e-01, 0,
                7.07106781e-01, 0, -7.07106781e-01, 0, 7.07106781e-01, 0,
                -7.07106781e-01, 0, 7.07106781e-01,
             7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
                7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
                7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
                7.07106781e-01, 7.07106781e-01, 7.07106781e-01, 7.07106781e-01,
             -2.48246554e-03, -6.63050529e-01, -2.48246716e-03, 6.58085596e-01,
                -3.18547255e-03, -6.63764473e-01, -3.18547463e-03,
                6.57393526e-01, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 3.88139002e-03, -6.56686671e-01,
                3.88139255e-03, 6.64449453e-01, 3.17838301e-03,
                -6.57400615e-01, 3.17838509e-03, 6.63757383e-01,
             2.48246635e-03, -1.22722488e-01, -2.48246635e-03, 1.22722488e-01,
                3.18547359e-03, 6.03708658e-04, -3.18547359e-03,
                -6.03708658e-04, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, -3.88139128e-03, -1.22686211e-01,
                3.88139128e-03, 1.22686211e-01, -3.17838405e-03,
                6.39985119e-04, 3.17838405e-03, -6.39985119e-04,
             7.19625056e-01, 1.19303674e-01, -4.74212371e-01, 1.26109011e-01,
                5.96307834e-01, -4.02458816e-03, -5.97551548e-01,
                2.78087418e-03, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 7.19625056e-01, 1.19303674e-01,
                -4.74212371e-01, 1.26109011e-01, 5.96307834e-01,
                -4.02458816e-03, -5.97551548e-01, 2.78087418e-03,
             6.00388727e-01, 6.68168393e-02, -4.60706394e-01, 7.28654938e-02,
                4.77071504e-01, -5.65114231e-02, -5.84045571e-01,
                -5.04626435e-02, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 6.00388727e-01, 6.68168393e-02,
                -4.60706394e-01, 7.28654938e-02, 4.77071504e-01,
                -5.65114231e-02, -5.84045571e-01, -5.04626435e-02,
             5.50122781e-01, 7.11606476e-02, -4.02371896e-01, 7.65902374e-02,
                4.26805559e-01, -5.21676148e-02, -5.25711073e-01,
                -4.67378999e-02, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 5.50122781e-01, 7.11606476e-02,
                -4.02371896e-01, 7.65902374e-02, 4.26805559e-01,
                -5.21676148e-02, -5.25711073e-01, -4.67378999e-02,
             5.25519921e-01, 2.77427750e-01, 3.21479905e-02, 2.80240162e-01,
                4.02202698e-01, 1.54099487e-01, -9.11911866e-02,
                1.56912024e-01, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 5.25519921e-01, 2.77427750e-01,
                3.21479905e-02, 2.80240162e-01, 4.02202698e-01,
                1.54099487e-01, -9.11911866e-02, 1.56912024e-01,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             1.15582432e-01, 7.54036567e-02, 3.56803542e-02, 7.58591296e-02,
                -7.73479056e-03, -4.79246057e-02, -8.76588229e-02,
                -4.74690077e-02, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 1.15582432e-01, 7.54036567e-02,
                3.56803542e-02, 7.58591296e-02, -7.73479056e-03,
                -4.79246057e-02, -8.76588229e-02, -4.74690077e-02;
  EXPECT_TRUE(
      CompareMatrices(B_q, B_real, 1e-8, MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(JB_q, JB_q_real, 1e-8, MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(B_v, B_real, 1e-8, MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(JB_v, JB_v_real, 1e-8, MatrixCompareType::relative));
}

}  // namespace
}  // namespace control
}  // namespace cassie
