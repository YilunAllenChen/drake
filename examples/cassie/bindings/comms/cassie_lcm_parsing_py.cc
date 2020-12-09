#include "cassie/comms/cassie_lcm_parsing.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;

namespace cassie {
namespace {

PYBIND11_MODULE(cassie_lcm_parsing, m) {
  m.doc() = "Bindings for cassie_lcm_parsing methods";

  m.attr("kCassieMotorToState") = py::cast(kCassieMotorToState);
  m.attr("kCassieJointToState") = py::cast(kCassieJointToState);
  m.attr("kCassieSpringToState") = py::cast(kCassieSpringToState);
  m.attr("kCassieMotorUrdfToSimulink") = py::cast(kCassieMotorUrdfToSimulink);
}

}  // namespace
}  // namespace cassie
