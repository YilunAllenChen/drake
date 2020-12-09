#include "cassie/cassie_common.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;

namespace cassie {
namespace {

PYBIND11_MODULE(cassie_common, m) {
  m.doc() = "Binding for cassie_common methods";

  m.attr("kCassiePositions") = py::cast(kCassiePositions);
  m.attr("kCassieVelocities") = py::cast(kCassieVelocities);
  m.attr("kCassieStates") = py::cast(kCassieStates);
  m.attr("kCassieActuators") = py::cast(kCassieActuators);
  m.attr("kCassieFourBarDistance") = py::cast(kCassieFourBarDistance);
  m.attr("p_midfoot") = py::cast(p_midfoot);

  py::enum_<CassieURDFType>(m, "CassieURDFType")
      .value("kStandardCassie", CassieURDFType::kStandardCassie)
      .value("kSoftSpringsCassie", CassieURDFType::kSoftSpringsCassie)
      .value("kActiveSpringsCassie", CassieURDFType::kActiveSpringsCassie)
      .value("kActiveAnkleCassie", CassieURDFType::kActiveAnkleCassie)
      .value("kFixedSpringCassie", CassieURDFType::kFixedSpringCassie);

  m.def("CassieFixedPointState",
        [](FloatingBaseType floating_base) {
          return CassieFixedPointState(floating_base);
        },
        py::arg("floating_base") = FloatingBaseType::kQuaternion);
  m.def("CassieFixedPointTorque", &CassieFixedPointTorque);
  m.def("GetFourBarHipMountPoint", &GetFourBarHipMountPoint);
  m.def("GetFourBarHeelMountPoint", &GetFourBarHeelMountPoint);

  m.def("BaseStateFromFullState", &BaseStateFromFullState,
        py::arg("CassieState"));
  m.def("getCassieTree",
        [](CassieURDFType urdf_id, FloatingBaseType floating_base) {
          return getCassieTree<double>(urdf_id, floating_base);
        },
        py::arg("urdf_id") = CassieURDFType::kStandardCassie,
        py::arg("floating_base") = FloatingBaseType::kQuaternion);
  m.def("setDefaultContactParams",
        [](RigidBodyPlant<double> &plant) {
          setDefaultContactParams(plant);
        },
        py::arg("plant"));
}

}  // namespace
}  // namespace cassie
