#pragma once

#include "cassie_types/lcmt_cassie_command.hpp"
#include "cassie_types/lcmt_cassie_state.hpp"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace cassie {
namespace control {
using drake::lcm::DrakeLcm;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::InputPort;
using drake::systems::LeafSystem;
using drake::systems::OutputPort;

//---------- StateReceiver class --------------------------
class StateReceiver : public LeafSystem<double> {
 public:
  StateReceiver();

  const OutputPort<double> &get_utime_output_port() const {
    return get_output_port(0);
  }

  const OutputPort<double> &get_position_output_port() const {
    return get_output_port(1);
  }

  const OutputPort<double> &get_velocity_output_port() const {
    return get_output_port(2);
  }

  const OutputPort<double> &get_contact_output_port() const {
    return get_output_port(3);
  }

 protected:
  drake::optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return true;
  }

 private:
  void OutputUtime(const Context<double> &context,
                   BasicVector<double> *output) const;
  void OutputPositions(const Context<double> &context,
                       BasicVector<double> *output) const;
  void OutputVelocities(const Context<double> &context,
                        BasicVector<double> *output) const;
  void OutputContact(const Context<double> &context,
                     BasicVector<double> *output) const;
};

//---------- CommandSender class --------------------------
class CommandSender : public LeafSystem<double> {
 public:
  CommandSender();

  const InputPort<double> &get_utime_input_port() const {
    return get_input_port(0);
  }

  const InputPort<double> &get_torque_input_port() const {
    return get_input_port(1);
  }

  const InputPort<double> &get_motor_pos_input_port() const {
    return get_input_port(2);
  }

  const InputPort<double> &get_motor_vel_input_port() const {
    return get_input_port(3);
  }

  const InputPort<double> &get_kp_input_port() const {
    return get_input_port(4);
  }

  const InputPort<double> &get_kd_input_port() const {
    return get_input_port(5);
  }

  const InputPort<double> &get_ki_input_port() const {
    return get_input_port(6);
  }

  const InputPort<double> &get_leak_input_port() const {
    return get_input_port(7);
  }

  const InputPort<double> &get_clamp_input_port() const {
    return get_input_port(8);
  }

 protected:
  drake::optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return true;
  }

 private:
  void OutputCommand(const Context<double> &context,
                     cassie_types::lcmt_cassie_command *msg) const;
};

StateReceiver *addLcmStateReceiver(DiagramBuilder<double> *builder,
                                   DrakeLcm *lcm);

CommandSender *addLcmCommandSender(DiagramBuilder<double> *builder,
                                   DrakeLcm *lcm, double publish_preiod);

}  // namespace control
}  // namespace cassie
