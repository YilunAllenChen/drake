#include "cassie/cassie_common.h"
#include "cassie/comms/cassie_lcm_parsing.h"
#include "cassie/control/lcm/control_comms.h"
#include "cassie_types/lcmt_cassie_command.hpp"
#include "cassie_types/lcmt_cassie_state.hpp"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include <gtest/gtest.h>

namespace cassie {
namespace control {
namespace {
using drake::systems::AbstractValue;
using drake::systems::BasicVector;
using drake::systems::Value;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

GTEST_TEST(LcmControlComms, StateRecieverPortAccess) {
  StateReceiver state_rec;
  auto context = state_rec.CreateDefaultContext();
  EXPECT_EQ(state_rec.get_num_input_ports(), 1);
  EXPECT_EQ(context->get_num_input_ports(), 1);
  EXPECT_EQ(state_rec.get_num_output_ports(), 4);
  EXPECT_EQ(context->get_num_output_ports(), 4);
  EXPECT_EQ(state_rec.get_utime_output_port().get_index(), 0);
  EXPECT_EQ(state_rec.get_utime_output_port().size(), 1);
  EXPECT_EQ(state_rec.get_position_output_port().get_index(), 1);
  EXPECT_EQ(state_rec.get_position_output_port().size(), kCassiePositions);
  EXPECT_EQ(state_rec.get_velocity_output_port().get_index(), 2);
  EXPECT_EQ(state_rec.get_velocity_output_port().size(), kCassieVelocities);
  EXPECT_EQ(state_rec.get_contact_output_port().get_index(), 3);
  EXPECT_EQ(state_rec.get_contact_output_port().size(), 2);
}

GTEST_TEST(LcmControlComms, StateReceiver) {
  const int kTime = 123456;
  StateReceiver state_rec;

  auto context = state_rec.CreateDefaultContext();
  auto out_utime = state_rec.get_output_port(0).Allocate();
  auto out_q = state_rec.get_output_port(1).Allocate();
  auto out_v = state_rec.get_output_port(2).Allocate();
  auto out_contact = state_rec.get_output_port(3).Allocate();

  cassie_types::lcmt_cassie_state msg;
  msg.utime = kTime;
  for (int ii = 0; ii < kCassiePositions; ii++) {
    msg.q[ii] = static_cast<float>(ii);
  }
  for (int ii = 0; ii < kCassieVelocities; ii++) {
    msg.v[ii] = static_cast<float>(kCassiePositions + ii);
  }
  msg.left_foot = true;
  msg.right_foot = false;

  context->FixInputPort(
      0, std::make_unique<Value<cassie_types::lcmt_cassie_state>>(msg));

  state_rec.get_output_port(0).Calc(*context, out_utime.get());
  const auto &output_vec_utime =
      out_utime->GetValueOrThrow<BasicVector<double>>();
  EXPECT_EQ(output_vec_utime.get_value()[0], kTime);

  state_rec.get_output_port(1).Calc(*context, out_q.get());
  const auto &output_vec_q = out_q->GetValueOrThrow<BasicVector<double>>();
  for (int ii = 0; ii < kCassiePositions; ii++) {
    EXPECT_EQ(output_vec_q.get_value()[ii], ii);
  }

  state_rec.get_output_port(2).Calc(*context, out_v.get());
  const auto &output_vec_v = out_v->GetValueOrThrow<BasicVector<double>>();
  for (int ii = 0; ii < kCassieVelocities; ii++) {
    EXPECT_EQ(output_vec_v.get_value()[ii], kCassiePositions + ii);
  }

  state_rec.get_output_port(3).Calc(*context, out_contact.get());
  const auto &output_vec_contact =
      out_contact->GetValueOrThrow<BasicVector<double>>();
  EXPECT_TRUE(output_vec_contact.get_value()[0]);
  EXPECT_FALSE(output_vec_contact.get_value()[1]);
}

GTEST_TEST(LcmControlComms, CommandSenderPortAccess) {
  CommandSender command_pub;
  auto context = command_pub.CreateDefaultContext();
  EXPECT_EQ(command_pub.get_num_input_ports(), 9);
  EXPECT_EQ(context->get_num_input_ports(), 9);
  EXPECT_EQ(command_pub.get_num_output_ports(), 1);
  EXPECT_EQ(context->get_num_output_ports(), 1);
  EXPECT_EQ(command_pub.get_utime_input_port().get_index(), 0);
  EXPECT_EQ(command_pub.get_utime_input_port().size(), 1);
  EXPECT_EQ(command_pub.get_torque_input_port().get_index(), 1);
  EXPECT_EQ(command_pub.get_torque_input_port().size(), kCassieActuators);
  EXPECT_EQ(command_pub.get_motor_pos_input_port().get_index(), 2);
  EXPECT_EQ(command_pub.get_motor_pos_input_port().size(), kCassieActuators);
  EXPECT_EQ(command_pub.get_motor_vel_input_port().get_index(), 3);
  EXPECT_EQ(command_pub.get_motor_vel_input_port().size(), kCassieActuators);
  EXPECT_EQ(command_pub.get_kp_input_port().get_index(), 4);
  EXPECT_EQ(command_pub.get_kp_input_port().size(), kCassieActuators);
  EXPECT_EQ(command_pub.get_kd_input_port().get_index(), 5);
  EXPECT_EQ(command_pub.get_kd_input_port().size(), kCassieActuators);
  EXPECT_EQ(command_pub.get_ki_input_port().get_index(), 6);
  EXPECT_EQ(command_pub.get_ki_input_port().size(), kCassieActuators);
  EXPECT_EQ(command_pub.get_leak_input_port().get_index(), 7);
  EXPECT_EQ(command_pub.get_leak_input_port().size(), kCassieActuators);
  EXPECT_EQ(command_pub.get_clamp_input_port().get_index(), 8);
  EXPECT_EQ(command_pub.get_clamp_input_port().size(), kCassieActuators);
}

GTEST_TEST(LcmControlComms, CommandSender) {
  const int kTime = 123456;
  CommandSender command_pub;

  auto context = command_pub.CreateDefaultContext();
  auto output = command_pub.AllocateOutput();
  auto input_utime = std::make_unique<BasicVector<double>>(1);
  auto input_torque = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_pos = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_vel = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_kp = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_kd = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_ki = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_leak = std::make_unique<BasicVector<double>>(kCassieActuators);
  auto input_clamp = std::make_unique<BasicVector<double>>(kCassieActuators);

  input_utime->get_mutable_value() << kTime;
  Eigen::MatrixXd inputs(8, kCassieActuators);
  for (int ii = 0; ii < 8; ii++) {
    for (int jj = 0; jj < kCassieActuators; jj++) {
      inputs(ii, jj) = ii * kCassieActuators + jj;
    }
  }
  input_torque->set_value(inputs.row(0));
  input_pos->set_value(inputs.row(1));
  input_vel->set_value(inputs.row(2));
  input_kp->set_value(inputs.row(3));
  input_kd->set_value(inputs.row(4));
  input_ki->set_value(inputs.row(5));
  input_leak->set_value(inputs.row(6));
  input_clamp->set_value(inputs.row(7));

  context->FixInputPort(0, std::move(input_utime));
  context->FixInputPort(1, std::move(input_torque));
  context->FixInputPort(2, std::move(input_pos));
  context->FixInputPort(3, std::move(input_vel));
  context->FixInputPort(4, std::move(input_kp));
  context->FixInputPort(5, std::move(input_kd));
  context->FixInputPort(6, std::move(input_ki));
  context->FixInputPort(7, std::move(input_leak));
  context->FixInputPort(8, std::move(input_clamp));

  command_pub.CalcOutput(*context, output.get());
  const AbstractValue *out_data = output->get_data(0);
  const auto &command = out_data->GetValue<cassie_types::lcmt_cassie_command>();

  EXPECT_EQ(command.utime, kTime);
  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(command.motor_torques[ii],
              (float)inputs(0, kCassieMotorUrdfToSimulink[ii]));
  }
  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(command.motor_positions_desired[ii],
              (float)inputs(1, kCassieMotorUrdfToSimulink[ii]));
  }
  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(command.motor_velocities_desired[ii],
              (float)inputs(2, kCassieMotorUrdfToSimulink[ii]));
  }
  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(command.motor_kp[ii],
              (float)inputs(3, kCassieMotorUrdfToSimulink[ii]));
  }
  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(command.motor_kd[ii],
              (float)inputs(4, kCassieMotorUrdfToSimulink[ii]));
  }
  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(command.motor_ki[ii],
              (float)inputs(5, kCassieMotorUrdfToSimulink[ii]));
  }
  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(command.leak_factor[ii],
              (float)inputs(6, kCassieMotorUrdfToSimulink[ii]));
  }
  for (int ii = 0; ii < kCassieActuators; ii++) {
    EXPECT_EQ(command.integrator_clamp[ii],
              (float)inputs(7, kCassieMotorUrdfToSimulink[ii]));
  }
}

GTEST_TEST(LcmControlComms, addLcmStateReceiver) {
  DiagramBuilder<double> builder;
  DrakeLcm lcm;
  StateReceiver *state_rec = addLcmStateReceiver(&builder, &lcm);

  EXPECT_FALSE(state_rec == nullptr);
  EXPECT_FALSE(builder.empty());
  auto systems = builder.GetMutableSystems();
  EXPECT_EQ(systems.size(), 2);
  EXPECT_EQ(typeid(*systems[0]), typeid(LcmSubscriberSystem));
  EXPECT_EQ(typeid(*systems[1]), typeid(StateReceiver));
}

GTEST_TEST(LcmControlComms, addLcmCommandSender) {
  DiagramBuilder<double> builder;
  DrakeLcm lcm;
  CommandSender *command_send = addLcmCommandSender(&builder, &lcm, 0.0005);

  EXPECT_FALSE(command_send == nullptr);
  EXPECT_FALSE(builder.empty());
  auto systems = builder.GetMutableSystems();
  EXPECT_EQ(systems.size(), 2);
  EXPECT_EQ(typeid(*systems[0]), typeid(LcmPublisherSystem));
  EXPECT_EQ(typeid(*systems[1]), typeid(CommandSender));
}

}  // namespace
}  // namespace control
}  // namespace cassie
