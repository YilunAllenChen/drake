#pragma once

#include "cassie/cassie_common.h"
#include "drake/multibody/rigid_body_tree.h"

namespace cassie {
namespace control {
using Eigen::Matrix;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// TODO: don't make this stuff hard coded
constexpr int m_surface_tangents = 4;
constexpr double mu = 1.0;

// TODO: This header assumes dim = 3, should that be fixed?
Matrix3Xd forwardKinTerrainPoints(const RigidBodyTree<double> &r,
                                  KinematicsCache<double> &cache,
                                  RigidBody<double> &lfoot,
                                  RigidBody<double> &rfoot);

MatrixXd contactJacobian(const RigidBodyTree<double> &r,
                         KinematicsCache<double> &cache,
                         RigidBody<double> &lfoot, RigidBody<double> &rfoot,
                         bool in_terms_of_qdot);

VectorXd contactJacobianDotTimesV(const RigidBodyTree<double> &r,
                                  KinematicsCache<double> &cache,
                                  RigidBody<double> &lfoot,
                                  RigidBody<double> &rfoot);

void contactDistancesAndNormals(const RigidBodyTree<double> &r,
                                KinematicsCache<double> &cache,
                                RigidBody<double> &lfoot,
                                RigidBody<double> &rfoot, VectorXd &phi,
                                Matrix3Xd &normals);

Matrix<double, 3, m_surface_tangents> surfaceTangents(Vector3d normal);

void contactJacobianBV(const RigidBodyTree<double> &r,
                       KinematicsCache<double> &cache, RigidBody<double> &lfoot,
                       RigidBody<double> &rfoot, bool in_terms_of_qdot,
                       Matrix3Xd &B, MatrixXd &JB);

}  // namespace control
}  // namespace cassie
