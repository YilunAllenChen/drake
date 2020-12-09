#include "cassie/control/control_util.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;

namespace cassie {
namespace control {
namespace {

PYBIND11_MODULE(control_util, m) {
  m.doc() = "Bindings for control_util methods";

  m.attr("mu") = py::cast(mu);
  m.attr("m_surface_tangents") = py::cast(m_surface_tangents);

  m.def("forwardKinTerrainPoints", &forwardKinTerrainPoints, py::arg("rtree"),
        py::arg("cache"), py::arg("lfoot"), py::arg("rfoot"));
  m.def("contactJacobian", &contactJacobian, py::arg("rtree"), py::arg("cache"),
        py::arg("lfoot"), py::arg("rfoot"), py::arg("in_terms_of_qdot"));
  m.def("contactJacobianDotTimesV", &contactJacobianDotTimesV, py::arg("rtree"),
        py::arg("cache"), py::arg("lfoot"), py::arg("rfoot"));

  m.def("surfaceTangents", &surfaceTangents);

  m.def("contactDistancesAndNormals",
        [](const RigidBodyTree<double> &rtree, KinematicsCache<double> &cache,
           RigidBody<double> &lfoot, RigidBody<double> &rfoot) {
          VectorXd phi;
          Matrix3Xd normals;
          contactDistancesAndNormals(rtree, cache, lfoot, rfoot, phi, normals);
          return py::make_tuple(phi, normals);
        },
        py::arg("rtree"), py::arg("cache"), py::arg("lfoot"), py::arg("rfoot"));
  m.def("contactJacobianBV",
        [](const RigidBodyTree<double> &rtree, KinematicsCache<double> &cache,
           RigidBody<double> &lfoot, RigidBody<double> &rfoot,
           bool in_terms_of_qdot) {
          Matrix3Xd B;
          MatrixXd JB;
          contactJacobianBV(rtree, cache, lfoot, rfoot, in_terms_of_qdot, B,
                            JB);
          return py::make_tuple(B, JB);
        },
        py::arg("rtree"), py::arg("cache"), py::arg("lfoot"), py::arg("rfoot"),
        py::arg("in_terms_of_qdot"));
}

}  // namespace
}  // namespace control
}  // namespace cassie
