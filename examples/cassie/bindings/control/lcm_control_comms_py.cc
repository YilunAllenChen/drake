#include "cassie/control/lcm/control_comms.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;

namespace cassie {
namespace control {
namespace {

PYBIND11_MODULE(lcm_control_comms, m) {
  m.doc() = "Bindings for lcm_control_comms methods";

  py::class_<StateReceiver, LeafSystem<double>> lcm_state_rec(
      m, "StateReceiver");
  lcm_state_rec
      .def(py::init<>())
      .def("get_utime_output_port", &StateReceiver::get_utime_output_port,
           py::return_value_policy::reference_internal)
      .def("get_position_output_port",
           &StateReceiver::get_position_output_port,
           py::return_value_policy::reference_internal)
      .def("get_velocity_output_port",
           &StateReceiver::get_velocity_output_port,
           py::return_value_policy::reference_internal)
      .def("get_contact_output_port",
           &StateReceiver::get_contact_output_port,
           py::return_value_policy::reference_internal);

  py::class_<CommandSender, LeafSystem<double>> lcm_command_pub(
      m, "CommandSender");
  lcm_command_pub
      .def(py::init<>())
      .def("get_utime_input_port", &CommandSender::get_utime_input_port,
           py::return_value_policy::reference_internal)
      .def("get_torque_input_port", &CommandSender::get_torque_input_port,
           py::return_value_policy::reference_internal)
      .def("get_motor_pos_input_port",
           &CommandSender::get_motor_pos_input_port,
           py::return_value_policy::reference_internal)
      .def("get_motor_vel_input_port",
           &CommandSender::get_motor_vel_input_port,
           py::return_value_policy::reference_internal)
      .def("get_kp_input_port", &CommandSender::get_kp_input_port,
           py::return_value_policy::reference_internal)
      .def("get_kd_input_port", &CommandSender::get_kd_input_port,
           py::return_value_policy::reference_internal)
      .def("get_ki_input_port", &CommandSender::get_ki_input_port,
           py::return_value_policy::reference_internal)
      .def("get_leak_input_port", &CommandSender::get_leak_input_port,
           py::return_value_policy::reference_internal)
      .def("get_clamp_input_port", &CommandSender::get_clamp_input_port,
           py::return_value_policy::reference_internal);

  m.def("addLcmStateReceiver", &addLcmStateReceiver, py::arg("builder"),
        py::arg("lcm"), py::return_value_policy::reference_internal);

  m.def("addLcmCommandSender", &addLcmCommandSender, py::arg("builder"),
        py::arg("lcm"), py::arg("publish_preiod"),
        py::return_value_policy::reference_internal);

  m.def("addLcmControlComms",
        [](DiagramBuilder<double> *builder, DrakeLcm *lcm,
           double publish_preiod) {
          StateReceiver *state_rec = addLcmStateReceiver(builder, lcm);
          CommandSender *command_pub =
              addLcmCommandSender(builder, lcm, publish_preiod);
          return py::make_tuple(state_rec, command_pub);
        },
        py::arg("builder"), py::arg("lcm"), py::arg("publish_preiod"),
        py::return_value_policy::reference_internal);
}

}  // namespace
}  // namespace control
}  // namespace cassie
