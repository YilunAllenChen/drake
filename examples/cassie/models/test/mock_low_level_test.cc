#include "cassie/cassie_common.h"
#include "cassie/comms/cassie_lcm_parsing.h"
#include "cassie/models/mock_low_level.h"
#include "cassie_types/lcmt_cassie_command.hpp"
#include "cassie_types/lcmt_cassie_sensor.hpp"
#include "cassie_types/lcmt_cassie_state.hpp"
#include "cassie_types/lcmt_cassie_status.hpp"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

#include <gtest/gtest.h>

namespace cassie {
namespace models {
namespace {
using drake::systems::AbstractValue;
using drake::systems::BasicVector;
using drake::systems::Value;

GTEST_TEST(MockLowLevel, mockCommandReceiverPortAccess) {
  mockCommandReceiver command_rec;
  auto context = command_rec.CreateDefaultContext();
  EXPECT_EQ(command_rec.get_num_input_ports(), 1);
  EXPECT_EQ(context->get_num_input_ports(), 1);
  EXPECT_EQ(command_rec.get_command_input_port().get_index(), 0);
  EXPECT_EQ(command_rec.get_num_output_ports(), 8);
  EXPECT_EQ(context->get_num_output_ports(), 8);
  EXPECT_EQ(command_rec.get_ff_output_port().get_index(), 0);
  EXPECT_EQ(command_rec.get_ff_output_port().size(), kCassieActuators);
  EXPECT_EQ(command_rec.get_q_d_output_port().get_index(), 1);
  EXPECT_EQ(command_rec.get_q_d_output_port().size(), kCassieActuators);
  EXPECT_EQ(command_rec.get_qdot_d_output_port().get_index(), 2);
  EXPECT_EQ(command_rec.get_qdot_d_output_port().size(), kCassieActuators);
  EXPECT_EQ(command_rec.get_kp_output_port().get_index(), 3);
  EXPECT_EQ(command_rec.get_kp_output_port().size(), kCassieActuators);
  EXPECT_EQ(command_rec.get_kd_output_port().get_index(), 4);
  EXPECT_EQ(command_rec.get_kd_output_port().size(), kCassieActuators);
  EXPECT_EQ(command_rec.get_ki_output_port().get_index(), 5);
  EXPECT_EQ(command_rec.get_ki_output_port().size(), kCassieActuators);
  EXPECT_EQ(command_rec.get_leak_output_port().get_index(), 6);
  EXPECT_EQ(command_rec.get_leak_output_port().size(), kCassieActuators);
  EXPECT_EQ(command_rec.get_clamp_output_port().get_index(), 7);
  EXPECT_EQ(command_rec.get_clamp_output_port().size(), kCassieActuators);
}

GTEST_TEST(MockLowLevel, mockCommandReceiver) {
  const int kTime = 123456;
  mockCommandReceiver command_rec;

  auto context = command_rec.CreateDefaultContext();
  auto out_torque = command_rec.get_output_port(0).Allocate();
  auto out_pos = command_rec.get_output_port(1).Allocate();
  auto out_vel = command_rec.get_output_port(2).Allocate();
  auto out_kp = command_rec.get_output_port(3).Allocate();
  auto out_kd = command_rec.get_output_port(4).Allocate();
  auto out_ki = command_rec.get_output_port(5).Allocate();
  auto out_leak = command_rec.get_output_port(6).Allocate();
  auto out_clamp = command_rec.get_output_port(7).Allocate();

  cassie_types::lcmt_cassie_command msg;
  msg.utime = kTime;
  for (int ii = 0; ii < kCassieActuators; ii++) {
    msg.motor_torques[ii] = static_cast<float>(ii);
    msg.motor_positions_desired[ii] = static_cast<float>(ii + kCassieActuators);
    msg.motor_velocities_desired[ii] =
        static_cast<float>(ii + 2 * kCassieActuators);
    msg.motor_kp[ii] = static_cast<float>(ii + 3 * kCassieActuators);
    msg.motor_kd[ii] = static_cast<float>(ii + 4 * kCassieActuators);
    msg.motor_ki[ii] = static_cast<float>(ii + 5 * kCassieActuators);
    msg.leak_factor[ii] = static_cast<float>(ii + 6 * kCassieActuators);
    msg.integrator_clamp[ii] = static_cast<float>(ii + 7 * kCassieActuators);
  }

  context->FixInputPort(
      0, std::make_unique<Value<cassie_types::lcmt_cassie_command>>(msg));

  command_rec.get_output_port(0).Calc(*context, out_torque.get());
  command_rec.get_output_port(1).Calc(*context, out_pos.get());
  command_rec.get_output_port(2).Calc(*context, out_vel.get());
  command_rec.get_output_port(3).Calc(*context, out_kp.get());
  command_rec.get_output_port(4).Calc(*context, out_kd.get());
  command_rec.get_output_port(5).Calc(*context, out_ki.get());
  command_rec.get_output_port(6).Calc(*context, out_leak.get());
  command_rec.get_output_port(7).Calc(*context, out_clamp.get());

  const auto &output_vec_torque =
      out_torque->GetValueOrThrow<BasicVector<double>>();
  const auto &output_vec_pos = out_pos->GetValueOrThrow<BasicVector<double>>();
  const auto &output_vec_vel = out_vel->GetValueOrThrow<BasicVector<double>>();
  const auto &output_vec_kp = out_kp->GetValueOrThrow<BasicVector<double>>();
  const auto &output_vec_kd = out_kd->GetValueOrThrow<BasicVector<double>>();
  const auto &output_vec_ki = out_ki->GetValueOrThrow<BasicVector<double>>();
  const auto &output_vec_leak =
      out_leak->GetValueOrThrow<BasicVector<double>>();
  const auto &output_vec_clamp =
      out_clamp->GetValueOrThrow<BasicVector<double>>();

  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(output_vec_torque.get_value()[ii], ii);
    EXPECT_EQ(output_vec_pos.get_value()[ii], ii + kCassieActuators);
    EXPECT_EQ(output_vec_vel.get_value()[ii], ii + 2 * kCassieActuators);
    EXPECT_EQ(output_vec_kp.get_value()[ii], ii + 3 * kCassieActuators);
    EXPECT_EQ(output_vec_kd.get_value()[ii], ii + 4 * kCassieActuators);
    EXPECT_EQ(output_vec_ki.get_value()[ii], ii + 5 * kCassieActuators);
    EXPECT_EQ(output_vec_leak.get_value()[ii], ii + 6 * kCassieActuators);
    EXPECT_EQ(output_vec_clamp.get_value()[ii], ii + 7 * kCassieActuators);
  }
}

GTEST_TEST(MockLowLevel, mockInputControllerPortAccess) {
  mockInputController input_ctrl;
  auto context = input_ctrl.CreateDefaultContext();
  EXPECT_EQ(input_ctrl.get_num_input_ports(), 9);
  EXPECT_EQ(context->get_num_input_ports(), 9);
  EXPECT_EQ(input_ctrl.get_ff_input_port().get_index(), 0);
  EXPECT_EQ(input_ctrl.get_ff_input_port().size(), kCassieActuators);
  EXPECT_EQ(input_ctrl.get_q_d_input_port().get_index(), 1);
  EXPECT_EQ(input_ctrl.get_q_d_input_port().size(), kCassieActuators);
  EXPECT_EQ(input_ctrl.get_qdot_d_input_port().get_index(), 2);
  EXPECT_EQ(input_ctrl.get_qdot_d_input_port().size(), kCassieActuators);
  EXPECT_EQ(input_ctrl.get_kp_input_port().get_index(), 3);
  EXPECT_EQ(input_ctrl.get_kp_input_port().size(), kCassieActuators);
  EXPECT_EQ(input_ctrl.get_kd_input_port().get_index(), 4);
  EXPECT_EQ(input_ctrl.get_kd_input_port().size(), kCassieActuators);
  EXPECT_EQ(input_ctrl.get_ki_input_port().get_index(), 5);
  EXPECT_EQ(input_ctrl.get_ki_input_port().size(), kCassieActuators);
  EXPECT_EQ(input_ctrl.get_leak_input_port().get_index(), 6);
  EXPECT_EQ(input_ctrl.get_leak_input_port().size(), kCassieActuators);
  EXPECT_EQ(input_ctrl.get_clamp_input_port().get_index(), 7);
  EXPECT_EQ(input_ctrl.get_clamp_input_port().size(), kCassieActuators);
  EXPECT_EQ(input_ctrl.get_state_input_port().get_index(), 8);
  EXPECT_EQ(input_ctrl.get_state_input_port().size(), kCassieStates);
  EXPECT_EQ(input_ctrl.get_num_output_ports(), 1);
  EXPECT_EQ(context->get_num_output_ports(), 1);
  EXPECT_EQ(input_ctrl.get_torque_output_port().get_index(), 0);
  EXPECT_EQ(input_ctrl.get_torque_output_port().size(), kCassieActuators);
}

// Currently only tests that torque is passed through
GTEST_TEST(MockLowLevel, mockInputController) {
  mockInputController ctrl;

  auto context = ctrl.CreateDefaultContext();
  auto output = ctrl.get_output_port(0).Allocate();
  auto input_torque = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_pos = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_vel = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_kp = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_kd = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_ki = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_leak = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_clamp = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_state = std::make_unique<BasicVector<double>>(kCassieStates);

  Eigen::VectorXd data_torque =
      Eigen::VectorXd::LinSpaced(kCassieActuators, 0, kCassieActuators - 1);
  Eigen::VectorXd data_zeros = Eigen::VectorXd::Zero(kCassieActuators);
  Eigen::VectorXd data_state =
      Eigen::VectorXd::LinSpaced(kCassieStates, 0, kCassieStates - 1);
  input_torque->set_value(data_torque);
  input_pos->set_value(data_zeros);
  input_vel->set_value(data_zeros);
  input_kp->set_value(data_zeros);
  input_kd->set_value(data_zeros);
  input_ki->set_value(data_zeros);
  input_leak->set_value(data_zeros);
  input_clamp->set_value(data_zeros);
  input_state->set_value(data_state);

  context->FixInputPort(0, std::move(input_torque));
  context->FixInputPort(1, std::move(input_pos));
  context->FixInputPort(2, std::move(input_vel));
  context->FixInputPort(3, std::move(input_kp));
  context->FixInputPort(4, std::move(input_kd));
  context->FixInputPort(5, std::move(input_ki));
  context->FixInputPort(6, std::move(input_leak));
  context->FixInputPort(7, std::move(input_clamp));
  context->FixInputPort(8, std::move(input_state));

  std::unique_ptr<DiscreteValues<double>> update =
      ctrl.AllocateDiscreteVariables();
  ctrl.CalcDiscreteVariableUpdates(*context, update.get());
  context->get_mutable_discrete_state_vector().set_value(
      update->get_mutable_vector().get_value());

  ctrl.get_output_port(0).Calc(*context, output.get());
  const auto &output_vec = output->GetValueOrThrow<BasicVector<double>>();
  for (int ii = 0; ii < kCassieActuators; ii++) {
    int sim_ii = kCassieMotorUrdfToSimulink[ii];
    EXPECT_EQ(output_vec.get_value()[sim_ii], data_torque(ii));
  }
}

GTEST_TEST(MockLowLevel, mockSensorSenderPortAccess) {
  mockSensorSender sensor_send;
  auto context = sensor_send.CreateDefaultContext();
  EXPECT_EQ(sensor_send.get_num_input_ports(), 2);
  EXPECT_EQ(context->get_num_input_ports(), 2);
  EXPECT_EQ(sensor_send.get_sensor_input_port().get_index(), 0);
  EXPECT_EQ(sensor_send.get_sensor_input_port().size(), kCassieStates);
  EXPECT_EQ(sensor_send.get_torque_input_port().get_index(), 1);
  EXPECT_EQ(sensor_send.get_torque_input_port().size(), kCassieActuators);
  EXPECT_EQ(sensor_send.get_num_output_ports(), 2);
  EXPECT_EQ(context->get_num_output_ports(), 2);
  EXPECT_EQ(sensor_send.get_sensor_output_port().get_index(), 0);
  EXPECT_EQ(sensor_send.get_status_output_port().get_index(), 1);
}

GTEST_TEST(MockLowLevel, mockSensorSender) {
  mockSensorSender sensor_send;

  auto context = sensor_send.CreateDefaultContext();
  auto output = sensor_send.AllocateOutput();
  auto input_state = std::make_unique<BasicVector<double>>(kCassieStates);
  auto input_torque = std::make_unique<BasicVector<double>>(kCassieActuators);

  Eigen::VectorXd state =
      Eigen::VectorXd::LinSpaced(kCassieStates, 0, kCassieStates - 1);
  Eigen::VectorXd torque = Eigen::VectorXd::LinSpaced(
      kCassieActuators, kCassieStates, kCassieStates + kCassieActuators - 1);
  input_state->set_value(state);
  input_torque->set_value(torque);
  context->FixInputPort(0, std::move(input_state));
  context->FixInputPort(1, std::move(input_torque));

  sensor_send.CalcOutput(*context, output.get());
  const AbstractValue *out_data = output->get_data(0);
  const auto &sensor = out_data->GetValue<cassie_types::lcmt_cassie_sensor>();
  out_data = output->get_data(1);
  const auto &status = out_data->GetValue<cassie_types::lcmt_cassie_status>();

  EXPECT_EQ(sensor.utime, status.utime);

  // Test sensor output
  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(sensor.motor_positions[ii],
              (float)state(kCassieMotorToState[ii]));
    EXPECT_EQ(sensor.motor_velocities[ii],
              (float)state(kCassiePositions + kCassieMotorToState[ii] - 1));
    int sim_ii = kCassieMotorUrdfToSimulink[ii];
    EXPECT_EQ(sensor.measured_torques[ii], (float)torque(sim_ii));
  }
  for (int ii = 0; ii < 4; ii++) {
    EXPECT_EQ(sensor.joint_positions[ii],
              (float)state(kCassieJointToState[ii]));
    EXPECT_EQ(sensor.joint_velocities[ii],
              (float)state(kCassiePositions + kCassieJointToState[ii] - 1));
    EXPECT_EQ(sensor.spring_deflections[ii], 0.0);
    EXPECT_EQ(sensor.spring_velocities[ii], 0.0);
    EXPECT_EQ(sensor.imu_orientation[ii], (float)state(ii + 3));
  }
  for (int ii = 0; ii < 3; ii++) {
    EXPECT_EQ(sensor.imu_linear_acceleration[ii], 0.0);
    EXPECT_EQ(sensor.imu_magnetic_field[ii], 0.0);
    EXPECT_EQ(sensor.imu_angular_velocity[ii],
              (float)state(kCassiePositions + ii));
  }

  // Test status output
  EXPECT_EQ(status.status, 0);
  EXPECT_EQ(status.charge_state, 100);
  EXPECT_EQ(status.pressure, 0.0);
  EXPECT_EQ(status.temperature, 0.0);
  for (int ii = 0; ii < 16; ii++) {
    EXPECT_EQ(status.radio[ii], 0.0);
  }
}

GTEST_TEST(MockLowLevel, mockStateSenderPortAccess) {
  mockStateSender state_send;
  auto context = state_send.CreateDefaultContext();
  EXPECT_EQ(state_send.get_num_input_ports(), 1);
  EXPECT_EQ(context->get_num_input_ports(), 1);
  EXPECT_EQ(state_send.get_state_input_port().get_index(), 0);
  EXPECT_EQ(state_send.get_state_input_port().size(), kCassieStates);
  EXPECT_EQ(state_send.get_num_output_ports(), 1);
  EXPECT_EQ(context->get_num_output_ports(), 1);
  EXPECT_EQ(state_send.get_state_output_port().get_index(), 0);
}

GTEST_TEST(MockLowLevel, mockStateSender) {
  mockStateSender state_send;

  auto context = state_send.CreateDefaultContext();
  auto output = state_send.AllocateOutput();
  auto input = std::make_unique<BasicVector<double>>(kCassieStates);

  Eigen::VectorXd data =
      Eigen::VectorXd::LinSpaced(kCassieStates, 0, kCassieStates - 1);
  input->set_value(data);
  context->FixInputPort(0, std::move(input));

  state_send.CalcOutput(*context, output.get());
  const AbstractValue *out_data = output->get_data(0);
  const auto &state = out_data->GetValue<cassie_types::lcmt_cassie_state>();

  // EXPECT_EQ(state.utime, kTime);
  for (int ii = 0; ii < kCassiePositions; ii++) {
    EXPECT_EQ(state.q[ii], (float)data(ii));
  }
  for (int ii = 0; ii < kCassieVelocities; ii++) {
    EXPECT_EQ(state.v[ii], (float)data(kCassiePositions + ii));
  }
  EXPECT_TRUE(state.left_foot);
  EXPECT_TRUE(state.right_foot);
}

}  // namespace
}  // namespace models
}  // namespace cassie
