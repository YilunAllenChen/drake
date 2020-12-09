#include "cassie/models/mock_low_level.h"
#include "cassie/cassie_common.h"
#include "cassie/comms/cassie_lcm_parsing.h"
#include <algorithm>

#include <vector>

namespace cassie {
namespace models {
using drake::systems::AbstractValue;
using drake::systems::kVectorValued;
using drake::systems::Value;

//---------- mockCommandReceiver functions ----------------
mockCommandReceiver::mockCommandReceiver() {
  this->DeclareAbstractInputPort("lcm-command",
                                 Value<cassie_types::lcmt_cassie_command>{});
  for (int ii = 0; ii < 8; ii++) {
    this->DeclareVectorOutputPort(
        BasicVector<double>(kCassieActuators),
        [this, ii](const Context<double> &c, BasicVector<double> *o) {
          this->OutputMsgData(c, o, ii);
        });
  }
}

void mockCommandReceiver::OutputMsgData(const Context<double> &context,
                                        BasicVector<double> *output,
                                        int port) const {
  const AbstractValue *input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto &command = input->GetValue<cassie_types::lcmt_cassie_command>();

  const float *command_data;
  switch (port) {
  case 0:
    command_data = command.motor_torques;
    // std::cout << context.get_time() * 1e6 - command.utime << std::endl;
    break;
  case 1:
    command_data = command.motor_positions_desired;
    break;
  case 2:
    command_data = command.motor_velocities_desired;
    break;
  case 3:
    command_data = command.motor_kp;
    break;
  case 4:
    command_data = command.motor_kd;
    break;
  case 5:
    command_data = command.motor_ki;
    break;
  case 6:
    command_data = command.leak_factor;
    break;
  case 7:
    command_data = command.integrator_clamp;
    break;
  default:
    command_data = nullptr;
  }

  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();

  for (int ii = 0; ii < kCassieActuators; ii++) {
    output_vec[ii] = static_cast<double>(command_data[ii]);
  }
}

//---------- mockInputController functions ----------------
mockInputController::mockInputController() {
  for (int ii = 0; ii < 8; ii++) {
    this->DeclareInputPort(kVectorValued, kCassieActuators);
  }
  this->DeclareInputPort(kVectorValued, kCassieStates);
  this->DeclareVectorOutputPort(BasicVector<double>(kCassieActuators),
                                &mockInputController::outputTorque);

  this->DeclarePeriodicDiscreteUpdate(0.0005);
  this->DeclareDiscreteState(kCassieActuators * 2);
}

void mockInputController::DoCalcDiscreteVariableUpdates(
    const Context<double> &context,
    const std::vector<const DiscreteUpdateEvent<double> *> &,
    DiscreteValues<double> *discrete_state) const {
  const auto ff_t =
      (VectorX<double>)this->EvalVectorInput(context, 0)->get_value();
  const auto q_d =
      (VectorX<double>)this->EvalVectorInput(context, 1)->get_value();
  const auto qdot_d =
      (VectorX<double>)this->EvalVectorInput(context, 2)->get_value();
  const auto kp =
      (VectorX<double>)this->EvalVectorInput(context, 3)->get_value();
  const auto kd =
      (VectorX<double>)this->EvalVectorInput(context, 4)->get_value();
  const auto ki =
      (VectorX<double>)this->EvalVectorInput(context, 5)->get_value();
  const auto leak =
      (VectorX<double>)this->EvalVectorInput(context, 6)->get_value();
  const auto clamp =
      (VectorX<double>)this->EvalVectorInput(context, 7)->get_value();
  const auto state =
      (VectorX<double>)this->EvalVectorInput(context, 8)->get_value();

  BasicVector<double> &command = discrete_state->get_mutable_vector(0);
  auto command_val = command.get_mutable_value();

  // Pull out the current state of the motors
  double dt = 0.0005;
  VectorX<double> q(10);
  VectorX<double> qdot(10);
  for (int ii = 0; ii < kCassieActuators; ii++) {
    q[ii] = state[kCassieMotorToState[ii]];
    qdot[ii] = state[kCassiePositions + kCassieMotorToState[ii] - 1];
  }

  // Calculate integral error
  for (int ii = 0; ii < kCassieActuators; ii++) {
    command_val[kCassieActuators + ii] += (q_d[ii] - q[ii]) * dt;
    command_val[kCassieActuators + ii] = std::max(
        std::min(leak[ii] * command_val[kCassieActuators + ii], clamp[ii]),
        -clamp[ii]);
  }

  // Calculate input
  for (int ii = 0; ii < kCassieActuators; ii++) {
    command_val[ii] = ff_t[ii] + kd[ii] * (qdot_d[ii] - qdot[ii]) +
                      kp[ii] * (q_d[ii] - q[ii]) +
                      ki[ii] * (command_val[kCassieActuators + ii]);
  }
}

void mockInputController::outputTorque(const Context<double> &context,
                                       BasicVector<double> *output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  VectorX<double> motor_in =
      context.get_discrete_state(0).get_value().head(kCassieActuators);
  for (int ii = 0; ii < kCassieActuators; ii++) {
    int sim_ii = kCassieMotorUrdfToSimulink[ii];
    output_vec[sim_ii] = motor_in[ii];
  }
}

//---------- mockSensorSender functions -------------------
mockSensorSender::mockSensorSender() {
  this->DeclareInputPort(kVectorValued, kCassieStates);
  this->DeclareInputPort(kVectorValued, kCassieActuators);
  this->DeclareAbstractOutputPort(&mockSensorSender::OutputSensor);
  this->DeclareAbstractOutputPort(&mockSensorSender::OutputStatus);
}

void mockSensorSender::OutputSensor(
    const Context<double> &context,
    cassie_types::lcmt_cassie_sensor *output) const {
  cassie_types::lcmt_cassie_sensor &msg = *output;

  const BasicVector<double> *state = this->EvalVectorInput(context, 0);
  const BasicVector<double> *torque = this->EvalVectorInput(context, 1);

  msg.utime = context.get_time() * 1e6;
  for (int ii = 0; ii < kCassieActuators; ii++) {
    msg.motor_positions[ii] = static_cast<float>(
        state->GetAtIndex(kCassieMotorToState[ii]));
    msg.motor_velocities[ii] = static_cast<float>(
        state->GetAtIndex(kCassiePositions + kCassieMotorToState[ii] - 1));
    int sim_ii = kCassieMotorUrdfToSimulink[ii];
    msg.measured_torques[ii] = static_cast<float>(torque->GetAtIndex(sim_ii));
  }
  for (int ii = 0; ii < 4; ii++) {
    msg.joint_positions[ii] = static_cast<float>(
        state->GetAtIndex(kCassieJointToState[ii]));
    msg.joint_velocities[ii] = static_cast<float>(
        state->GetAtIndex(kCassiePositions + kCassieJointToState[ii] - 1));
    // TODO: Implement spring deflection publishing
    msg.spring_deflections[ii] = 0.0;
    msg.spring_velocities[ii] = 0.0;
    msg.imu_orientation[ii] = static_cast<float>(state->GetAtIndex(ii + 3));
  }
  for (int ii = 0; ii < 3; ii++) {
    msg.imu_angular_velocity[ii] = static_cast<float>(
        state->GetAtIndex(kCassiePositions + ii));
    // TODO: Implement linear accel publishing
    msg.imu_linear_acceleration[ii] = 0.0;
    // TODO: Implement magnetic field publishing
    msg.imu_magnetic_field[ii] = 0.0;
  }
}

void mockSensorSender::OutputStatus(
    const Context<double> &context,
    cassie_types::lcmt_cassie_status *output) const {
  cassie_types::lcmt_cassie_status &msg = *output;

  // const BasicVector<double>* state = this->EvalVectorInput(context, 0);

  // TODO: Implement status publishing
  msg.utime = context.get_time() * 1e6;
  msg.status = 0;
  msg.charge_state = 100;
  msg.pressure = 0.0;
  msg.temperature = 0.0;
  for (int ii = 0; ii < 16; ii++) {
    msg.radio[ii] = 0.0;
  }
}

//---------- mockStateSender functions --------------------
mockStateSender::mockStateSender() {
  this->DeclareInputPort(kVectorValued, kCassieStates);
  this->DeclareAbstractOutputPort(&mockStateSender::OutputState);
}

void mockStateSender::OutputState(
    const Context<double> &context,
    cassie_types::lcmt_cassie_state *output) const {
  cassie_types::lcmt_cassie_state &msg = *output;

  const BasicVector<double> *state = this->EvalVectorInput(context, 0);

  msg.utime = context.get_time() * 1e6;
  for (int ii = 0; ii < kCassiePositions; ii++) {
    msg.q[ii] = static_cast<float>(state->GetAtIndex(ii));
  }
  for (int ii = 0; ii < kCassieVelocities; ii++) {
    msg.v[ii] = static_cast<float>(state->GetAtIndex(ii + kCassiePositions));
  }

  // TODO: Implement foot contact publishing
  msg.left_foot = true;
  msg.right_foot = true;
}

}  // namespace models
}  // namespace cassie
