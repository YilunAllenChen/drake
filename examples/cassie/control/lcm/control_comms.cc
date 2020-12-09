#include "cassie/control/lcm/control_comms.h"
#include "cassie/cassie_common.h"
#include "cassie/comms/cassie_lcm_parsing.h"
#include "cassie_types/lcmt_cassie_command.hpp"
#include "cassie_types/lcmt_cassie_state.hpp"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <memory>

namespace cassie {
namespace control {
using drake::lcm::DrakeLcm;
using drake::systems::AbstractValue;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::kVectorValued;
using drake::systems::LeafSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::Value;

//---------- StateReceiver class --------------------------
StateReceiver::StateReceiver() {
  this->DeclareAbstractInputPort("lcm-state",
                                 Value<cassie_types::lcmt_cassie_state>{});

  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &StateReceiver::OutputUtime);
  this->DeclareVectorOutputPort(BasicVector<double>(kCassiePositions),
                                &StateReceiver::OutputPositions);
  this->DeclareVectorOutputPort(BasicVector<double>(kCassieVelocities),
                                &StateReceiver::OutputVelocities);
  this->DeclareVectorOutputPort(BasicVector<double>(2),
                                &StateReceiver::OutputContact);
}

void StateReceiver::OutputUtime(const Context<double> &context,
                                BasicVector<double> *output) const {
  const AbstractValue *input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto &cassie_state = input->GetValue<cassie_types::lcmt_cassie_state>();

  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();

  output_vec[0] = static_cast<double>(cassie_state.utime);
}

void StateReceiver::OutputPositions(const Context<double> &context,
                                    BasicVector<double> *output) const {
  const AbstractValue *input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto &cassie_state = input->GetValue<cassie_types::lcmt_cassie_state>();

  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();

  for (int ii = 0; ii < kCassiePositions; ii++) {
    output_vec[ii] = static_cast<double>(cassie_state.q[ii]);
  }
}

void StateReceiver::OutputVelocities(const Context<double> &context,
                                     BasicVector<double> *output) const {
  const AbstractValue *input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto &cassie_state = input->GetValue<cassie_types::lcmt_cassie_state>();

  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();

  for (int ii = 0; ii < kCassieVelocities; ii++) {
    output_vec[ii] = static_cast<double>(cassie_state.v[ii]);
  }
}

void StateReceiver::OutputContact(const Context<double> &context,
                                  BasicVector<double> *output) const {
  const AbstractValue *input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto &cassie_state = input->GetValue<cassie_types::lcmt_cassie_state>();

  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();

  output_vec[0] = static_cast<double>(cassie_state.left_foot);
  output_vec[1] = static_cast<double>(cassie_state.right_foot);
}

//---------- CommandSender class --------------------------
CommandSender::CommandSender() {
  this->DeclareInputPort(kVectorValued, 1);
  for (int ii = 0; ii < 8; ii++) {
    this->DeclareInputPort(kVectorValued, kCassieActuators);
  }
  this->DeclareAbstractOutputPort(&CommandSender::OutputCommand);
}

void CommandSender::OutputCommand(
    const Context<double> &context,
    cassie_types::lcmt_cassie_command *msg) const {
  const BasicVector<double> *utime = this->EvalVectorInput(context, 0);
  const BasicVector<double> *torque = this->EvalVectorInput(context, 1);
  const BasicVector<double> *motor_pos = this->EvalVectorInput(context, 2);
  const BasicVector<double> *motor_vel = this->EvalVectorInput(context, 3);
  const BasicVector<double> *kp = this->EvalVectorInput(context, 4);
  const BasicVector<double> *kd = this->EvalVectorInput(context, 5);
  const BasicVector<double> *ki = this->EvalVectorInput(context, 6);
  const BasicVector<double> *leak = this->EvalVectorInput(context, 7);
  const BasicVector<double> *clamp = this->EvalVectorInput(context, 8);

  msg->utime = static_cast<float>(utime->GetAtIndex(0));
  for (int ii = 0; ii < kCassieActuators; ii++) {
    int sim_ii = kCassieMotorUrdfToSimulink[ii];
    msg->motor_torques[ii] = static_cast<float>(torque->GetAtIndex(sim_ii));
    msg->motor_positions_desired[ii] =
        static_cast<float>(motor_pos->GetAtIndex(sim_ii));
    msg->motor_velocities_desired[ii] =
        static_cast<float>(motor_vel->GetAtIndex(sim_ii));
    msg->motor_kp[ii] = static_cast<float>(kp->GetAtIndex(sim_ii));
    msg->motor_kd[ii] = static_cast<float>(kd->GetAtIndex(sim_ii));
    msg->motor_ki[ii] = static_cast<float>(ki->GetAtIndex(sim_ii));
    msg->leak_factor[ii] = static_cast<float>(leak->GetAtIndex(sim_ii));
    msg->integrator_clamp[ii] = static_cast<float>(clamp->GetAtIndex(sim_ii));
  }
}

//---------- Control LCM Diagrams -------------------------

StateReceiver *addLcmStateReceiver(DiagramBuilder<double> *builder,
                                   DrakeLcm *lcm) {
  auto state_sub = builder->AddSystem(
      LcmSubscriberSystem::Make<cassie_types::lcmt_cassie_state>("CASSIE_STATE",
                                                                 lcm));
  StateReceiver *state_rec =
      builder->AddSystem(std::make_unique<StateReceiver>());

  builder->Connect(state_sub->get_output_port(), state_rec->get_input_port(0));

  return state_rec;
}

CommandSender *addLcmCommandSender(DiagramBuilder<double> *builder,
                                   DrakeLcm *lcm, double publish_preiod) {
  auto command_pub = builder->AddSystem(
      LcmPublisherSystem::Make<cassie_types::lcmt_cassie_command>(
          "CASSIE_COMMAND", lcm));
  CommandSender *command_send =
      builder->AddSystem(std::make_unique<CommandSender>());

  command_pub->set_publish_period(publish_preiod);
  builder->Connect(command_send->get_output_port(0),
                   command_pub->get_input_port());

  return command_send;
}

}  // namespace control
}  // namespace cassie
