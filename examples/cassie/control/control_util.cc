#include "cassie/control/control_util.h"

#include <math.h>

namespace cassie {
namespace control {

Matrix3Xd forwardKinTerrainPoints(const RigidBodyTree<double> &r,
                                  KinematicsCache<double> &cache,
                                  RigidBody<double> &lfoot,
                                  RigidBody<double> &rfoot) {
  Matrix3Xd l_pts_foot;
  Matrix3Xd r_pts_foot;

  r.getTerrainContactPoints(lfoot, &l_pts_foot);
  r.getTerrainContactPoints(rfoot, &r_pts_foot);

  Matrix3Xd l_pts =
      r.transformPoints(cache, l_pts_foot, lfoot.get_body_index(), 0);
  Matrix3Xd r_pts =
      r.transformPoints(cache, r_pts_foot, rfoot.get_body_index(), 0);

  Matrix3Xd pts(3, l_pts.cols() + r_pts.cols());
  pts << l_pts, r_pts;

  return pts;
}

MatrixXd contactJacobian(const RigidBodyTree<double> &r,
                         KinematicsCache<double> &cache,
                         RigidBody<double> &lfoot, RigidBody<double> &rfoot,
                         bool in_terms_of_qdot) {
  Matrix3Xd l_pts_foot;
  Matrix3Xd r_pts_foot;

  r.getTerrainContactPoints(lfoot, &l_pts_foot);
  r.getTerrainContactPoints(rfoot, &r_pts_foot);

  MatrixXd l_J = r.transformPointsJacobian(
      cache, l_pts_foot, lfoot.get_body_index(), 0, in_terms_of_qdot);
  MatrixXd r_J = r.transformPointsJacobian(
      cache, r_pts_foot, rfoot.get_body_index(), 0, in_terms_of_qdot);

  MatrixXd J(l_J.rows() + r_J.rows(), l_J.cols());
  J << l_J, r_J;

  return J;
}

VectorXd contactJacobianDotTimesV(const RigidBodyTree<double> &r,
                                  KinematicsCache<double> &cache,
                                  RigidBody<double> &lfoot,
                                  RigidBody<double> &rfoot) {
  Matrix3Xd l_pts_foot;
  Matrix3Xd r_pts_foot;

  r.getTerrainContactPoints(lfoot, &l_pts_foot);
  r.getTerrainContactPoints(rfoot, &r_pts_foot);

  VectorXd l_Jdotv = r.transformPointsJacobianDotTimesV(
      cache, l_pts_foot, lfoot.get_body_index(), 0);
  VectorXd r_Jdotv = r.transformPointsJacobianDotTimesV(
      cache, r_pts_foot, rfoot.get_body_index(), 0);

  VectorXd Jdotv(l_Jdotv.size() + r_Jdotv.size());
  Jdotv << l_Jdotv, r_Jdotv;

  return Jdotv;
}

// assume flat terrain at z=0 for now
// TODO: deal with non-flat terrain/ terrain at diff heights
void contactDistancesAndNormals(const RigidBodyTree<double> &r,
                                KinematicsCache<double> &cache,
                                RigidBody<double> &lfoot,
                                RigidBody<double> &rfoot, VectorXd &phi,
                                Matrix3Xd &normals) {
  Matrix3Xd pts = forwardKinTerrainPoints(r, cache, lfoot, rfoot);
  int nc = pts.cols();

  phi.resize(nc);
  normals.resize(3, nc);
  phi = pts.row(2);
  for (int ii = 0; ii < nc; ii++) {
    normals.col(ii) << 0, 0, 1;
  }
}

Matrix<double, 3, m_surface_tangents> surfaceTangents(Vector3d normal) {
  const double kEpsilon = 1e-8;
  Vector3d t1;
  Vector3d t2;

  if (1 - normal[2] < kEpsilon) {
    t1 << 1, 0, 0;
  } else if (1 + normal[2] < kEpsilon) {
    t1 << -1, 0, 0;
  } else {
    t1 << normal[1], -normal[0], 0;
    t1.normalize();
  }

  t2 = t1.cross(normal);
  Matrix<double, 3, m_surface_tangents> d;
  for (int ii = 0; ii < m_surface_tangents; ii++) {
    double theta = ii * 2 * M_PI / m_surface_tangents;
    d.col(ii) = cos(theta) * t1 + sin(theta) * t2;
  }

  return d;
}

void contactJacobianBV(const RigidBodyTree<double> &r,
                       KinematicsCache<double> &cache, RigidBody<double> &lfoot,
                       RigidBody<double> &rfoot, bool in_terms_of_qdot,
                       Matrix3Xd &B, MatrixXd &JB) {
  MatrixXd J = contactJacobian(r, cache, lfoot, rfoot, in_terms_of_qdot);
  // assume flat terrain at z=0 for now
  Vector3d normal;
  normal << 0, 0, 1;
  Matrix<double, 3, m_surface_tangents> d = surfaceTangents(normal);

  int nc = J.rows() / 3;
  int ncols = nc * m_surface_tangents;
  int nrows;
  if (in_terms_of_qdot) {
    nrows = r.get_num_positions();
  } else {
    nrows = r.get_num_velocities();
  }

  B.resize(3, ncols);  // Friction polyhedron basis vectors
  JB.resize(nrows,
            ncols);  // Jacobian mapping forces along friction basis vectors

  double norm = sqrt(1.0 + mu * mu);

  for (int ii = 0; ii < nc; ii++) {
    Matrix3Xd sub_J(3, nrows);
    sub_J << J.row(3 * ii), J.row(3 * ii + 1), J.row(3 * ii + 2);
    for (int jj = 0; jj < m_surface_tangents; jj++) {
      B.col(ii * m_surface_tangents + jj) = (normal + mu * d.col(jj)) / norm;
      JB.col(ii * m_surface_tangents + jj) =
          sub_J.transpose() * B.col(ii * m_surface_tangents + jj);
    }
  }
}

}  // namespace control
}  // namespace cassie
