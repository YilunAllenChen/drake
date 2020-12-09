#pragma once

#include "cassie_types/lcmt_cassie_command.hpp"
#include "cassie_types/lcmt_cassie_sensor.hpp"
#include "cassie_types/lcmt_cassie_state.hpp"
#include "cassie_types/lcmt_cassie_status.hpp"
#include "drake/systems/framework/leaf_system.h"

#include <vector>

namespace cassie {
namespace models {
using drake::systems::LeafSystem;
using drake::systems::InputPort;
using drake::systems::OutputPort;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::BasicVector;

//---------- mockCommandReceiver class --------------------
class mockCommandReceiver : public LeafSystem<double> {
 public:
  mockCommandReceiver();

  const InputPort<double> &get_command_input_port() const {
    return this->get_input_port(0);
  }

  const OutputPort<double> &get_ff_output_port() const {
    return this->get_output_port(0);
  }

  const OutputPort<double> &get_q_d_output_port() const {
    return this->get_output_port(1);
  }

  const OutputPort<double> &get_qdot_d_output_port() const {
    return this->get_output_port(2);
  }

  const OutputPort<double> &get_kp_output_port() const {
    return this->get_output_port(3);
  }

  const OutputPort<double> &get_kd_output_port() const {
    return this->get_output_port(4);
  }

  const OutputPort<double> &get_ki_output_port() const {
    return this->get_output_port(5);
  }

  const OutputPort<double> &get_leak_output_port() const {
    return this->get_output_port(6);
  }

  const OutputPort<double> &get_clamp_output_port() const {
    return this->get_output_port(7);
  }

 protected:
  drake::optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return true;
  }

 private:
  void OutputMsgData(const Context<double> &context,
                     BasicVector<double> *output, int port) const;
};

//---------- mockInputController class --------------------
class mockInputController : public LeafSystem<double> {
 public:
  mockInputController();

  const InputPort<double> &get_ff_input_port() const {
    return this->get_input_port(0);
  }

  const InputPort<double> &get_q_d_input_port() const {
    return this->get_input_port(1);
  }

  const InputPort<double> &get_qdot_d_input_port() const {
    return this->get_input_port(2);
  }

  const InputPort<double> &get_kp_input_port() const {
    return this->get_input_port(3);
  }

  const InputPort<double> &get_kd_input_port() const {
    return this->get_input_port(4);
  }

  const InputPort<double> &get_ki_input_port() const {
    return this->get_input_port(5);
  }

  const InputPort<double> &get_leak_input_port() const {
    return this->get_input_port(6);
  }

  const InputPort<double> &get_clamp_input_port() const {
    return this->get_input_port(7);
  }

  const InputPort<double> &get_state_input_port() const {
    return this->get_input_port(8);
  }

  const OutputPort<double> &get_torque_output_port() const {
    return this->get_output_port(0);
  }

 protected:
  drake::optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

 private:
  void DoCalcDiscreteVariableUpdates(
      const Context<double> &context,
      const std::vector<const DiscreteUpdateEvent<double> *> &,
      DiscreteValues<double> *discrete_state) const override;

  void outputTorque(const Context<double> &context,
                    BasicVector<double> *output) const;
};

//---------- mockSensorSender class -----------------------
class mockSensorSender : public LeafSystem<double> {
 public:
  mockSensorSender();

  const InputPort<double> &get_sensor_input_port() const {
    return this->get_input_port(0);
  }

  const InputPort<double> &get_torque_input_port() const {
    return this->get_input_port(1);
  }

  const OutputPort<double> &get_sensor_output_port() const {
    return this->get_output_port(0);
  }

  const OutputPort<double> &get_status_output_port() const {
    return this->get_output_port(1);
  }

 private:
  void OutputSensor(const Context<double> &context,
                    cassie_types::lcmt_cassie_sensor *output) const;

  void OutputStatus(const Context<double> &context,
                    cassie_types::lcmt_cassie_status *output) const;
};

//---------- mockStateSender class ------------------------
class mockStateSender : public LeafSystem<double> {
 public:
  mockStateSender();

  const InputPort<double> &get_state_input_port() const {
    return this->get_input_port(0);
  }

  const OutputPort<double> &get_state_output_port() const {
    return this->get_output_port(0);
  }

 private:
  void OutputState(const Context<double> &context,
                   cassie_types::lcmt_cassie_state *output) const;
};

}  // namespace models
}  // namespace cassie
