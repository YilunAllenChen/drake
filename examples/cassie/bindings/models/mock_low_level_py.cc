#include "cassie/models/mock_low_level.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;

namespace cassie {
namespace models {
namespace {

PYBIND11_MODULE(mock_low_level, m) {
  m.doc() = "Bindings for mock_low_level methods";

  py::class_<mockCommandReceiver, LeafSystem<double>> mock_command_rec(
      m, "mockCommandReceiver");
  mock_command_rec
      .def(py::init<>())
      .def("get_command_input_port",
           &mockCommandReceiver::get_command_input_port,
           py::return_value_policy::reference_internal)
      .def("get_ff_output_port", &mockCommandReceiver::get_ff_output_port,
           py::return_value_policy::reference_internal)
      .def("get_q_d_output_port", &mockCommandReceiver::get_q_d_output_port,
           py::return_value_policy::reference_internal)
      .def("get_qdot_d_output_port",
           &mockCommandReceiver::get_qdot_d_output_port,
           py::return_value_policy::reference_internal)
      .def("get_kp_output_port", &mockCommandReceiver::get_kp_output_port,
           py::return_value_policy::reference_internal)
      .def("get_kd_output_port", &mockCommandReceiver::get_kd_output_port,
           py::return_value_policy::reference_internal)
      .def("get_ki_output_port", &mockCommandReceiver::get_ki_output_port,
           py::return_value_policy::reference_internal)
      .def("get_leak_output_port", &mockCommandReceiver::get_leak_output_port,
           py::return_value_policy::reference_internal)
      .def("get_clamp_output_port", &mockCommandReceiver::get_clamp_output_port,
           py::return_value_policy::reference_internal);

  py::class_<mockInputController, LeafSystem<double>> mock_input_ctrl(
      m, "mockInputController");
  mock_input_ctrl
      .def(py::init<>())
      .def("get_ff_input_port", &mockInputController::get_ff_input_port,
           py::return_value_policy::reference_internal)
      .def("get_q_d_input_port", &mockInputController::get_q_d_input_port,
           py::return_value_policy::reference_internal)
      .def("get_qdot_d_input_port", &mockInputController::get_qdot_d_input_port,
           py::return_value_policy::reference_internal)
      .def("get_kp_input_port", &mockInputController::get_kp_input_port,
           py::return_value_policy::reference_internal)
      .def("get_kd_input_port", &mockInputController::get_kd_input_port,
           py::return_value_policy::reference_internal)
      .def("get_ki_input_port", &mockInputController::get_ki_input_port,
           py::return_value_policy::reference_internal)
      .def("get_leak_input_port", &mockInputController::get_leak_input_port,
           py::return_value_policy::reference_internal)
      .def("get_clamp_input_port", &mockInputController::get_clamp_input_port,
           py::return_value_policy::reference_internal)
      .def("get_state_input_port", &mockInputController::get_state_input_port,
           py::return_value_policy::reference_internal)
      .def("get_torque_output_port",
           &mockInputController::get_torque_output_port,
           py::return_value_policy::reference_internal);

  py::class_<mockSensorSender, LeafSystem<double>> mock_sensor_send(
      m, "mockSensorSender");
  mock_sensor_send
      .def(py::init<>())
      .def("get_sensor_input_port", &mockSensorSender::get_sensor_input_port,
           py::return_value_policy::reference_internal)
      .def("get_torque_input_port", &mockSensorSender::get_torque_input_port,
           py::return_value_policy::reference_internal)
      .def("get_sensor_output_port", &mockSensorSender::get_sensor_output_port,
           py::return_value_policy::reference_internal)
      .def("get_status_output_port", &mockSensorSender::get_status_output_port,
           py::return_value_policy::reference_internal);

  py::class_<mockStateSender, LeafSystem<double>> mock_state_send(
      m, "mockStateSender");
  mock_state_send
      .def(py::init<>())
      .def("get_state_input_port", &mockStateSender::get_state_input_port,
           py::return_value_policy::reference_internal)
      .def("get_state_output_port", &mockStateSender::get_state_output_port,
           py::return_value_policy::reference_internal);
}

}  // namespace
}  // namespace models
}  // namespace cassie
