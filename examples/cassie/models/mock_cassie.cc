/// System to mock the real robot in simulation

#include "cassie/cassie_common.h"
#include "cassie/models/mock_low_level.h"
#include "cassie_types/lcmt_cassie_command.hpp"
#include "cassie_types/lcmt_cassie_sensor.hpp"
#include "cassie_types/lcmt_cassie_state.hpp"
#include "cassie_types/lcmt_cassie_status.hpp"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
// #include "drake/systems/primitives/constant_vector_source.h"

namespace cassie {
namespace {
using drake::manipulation::util::SimDiagramBuilder;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::DrakeVisualizer;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
// using drake::systems::ConstantVectorSource;

int doMain(std::string sim_type) {
  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;
  DiagramBuilder<double>* base_builder = builder.get_mutable_builder();

  const char* const kCassieSensorChannel = "CASSIE_SENSOR";
  const char* const kCassieStatusChannel  = "CASSIE_STATUS";
  const char* const kCassieStateChannel  = "CASSIE_STATE";
  const char* const kCassieCommandChannel  = "CASSIE_COMMAND";

  //---------- Set up LCM Subscriber ----------------------
  auto command_sub = base_builder->AddSystem(
        LcmSubscriberSystem::Make<cassie_types::lcmt_cassie_command>(
              kCassieCommandChannel, &lcm));

  //---------- Set up Comand Handler ----------------------
  auto command_rec = base_builder->AddSystem(
        std::make_unique<models::mockCommandReceiver>());

  //---------- Set up Input Controller --------------------
  auto input_ctrl = base_builder->AddSystem(
        std::make_unique<models::mockInputController>());

  //---------- Set up Cassie Simulation -------------------
  auto tree = getCassieTreed();
  auto plantDt =
      std::make_unique<RigidBodyPlant<double>>(std::move(tree), 0.0001);
  RigidBodyPlant<double>* plant = builder.AddPlant(std::move(plantDt));

  setDefaultContactParams(*plant);

  //---------- Set up Sensor/State Handler ----------------
  models::mockSensorSender* sensor_send = nullptr;
  models::mockStateSender*  state_send  = nullptr;

  // For sensor publishing
  if (sim_type == "sensor") {
    sensor_send = base_builder->AddSystem(
          std::make_unique<models::mockSensorSender>());
  }

  // For state publishing
  if (sim_type == "state") {
    state_send = base_builder->AddSystem(
          std::make_unique<models::mockStateSender>());
  }

  //---------- Set up LCM Publishers ----------------------
  LcmPublisherSystem* sensor_pub = nullptr;
  LcmPublisherSystem* status_pub = nullptr;
  LcmPublisherSystem* state_pub  = nullptr;

  // For sensor publishing
  if (sim_type == "sensor") {
    sensor_pub = base_builder->AddSystem(
          LcmPublisherSystem::Make<cassie_types::lcmt_cassie_sensor>(
                kCassieSensorChannel, &lcm));
    sensor_pub->set_publish_period(0.0005);
    status_pub = base_builder->AddSystem(
          LcmPublisherSystem::Make<cassie_types::lcmt_cassie_status>(
                kCassieStatusChannel, &lcm));
    status_pub->set_publish_period(0.005);
  }

  // For state publishing
  if (sim_type == "state") {
    state_pub  = base_builder->AddSystem(
          LcmPublisherSystem::Make<cassie_types::lcmt_cassie_state>(
                kCassieStateChannel, &lcm));
    state_pub->set_publish_period(0.0005);
  }

  //---------- Set up Visualizer --------------------------
  // Creates and adds LCM publisher for visualization.
  auto viz_tree = getCassieTreed();
  auto viz = base_builder->AddSystem<DrakeVisualizer>(*viz_tree, &lcm, true);
  viz->set_publish_period(1e-2);

  //---------- Wire up Diagram ----------------------------
  // For low-level control from commands
  base_builder->Connect(command_sub->get_output_port(),
                        command_rec->get_command_input_port());
  for (int ii = 0; ii < 8; ii++) {
    base_builder->Connect(command_rec->get_output_port(ii),
                          input_ctrl->get_input_port(ii));
  }
  base_builder->Connect(plant->get_output_port(0),
                        input_ctrl->get_state_input_port());
  base_builder->Connect(input_ctrl->get_torque_output_port(),
                        plant->get_input_port(0));

  // For sensor publishing
  if (sim_type == "sensor") {
    base_builder->Connect(plant->get_output_port(0),
                          sensor_send->get_sensor_input_port());
    base_builder->Connect(input_ctrl->get_torque_output_port(),
                          sensor_send->get_torque_input_port());
    base_builder->Connect(sensor_send->get_sensor_output_port(),
                          sensor_pub->get_input_port());
    base_builder->Connect(sensor_send->get_status_output_port(),
                          status_pub->get_input_port());
  }

  // For state publishing
  if (sim_type == "state") {
    base_builder->Connect(plant->get_output_port(0),
                          state_send->get_state_input_port());
    base_builder->Connect(state_send->get_state_output_port(),
                          state_pub->get_input_port());
  }

  base_builder->Connect(plant->get_output_port(0), viz->get_input_port(0));

  //---------- Run Mock Robot -----------------------------
  // VectorX<double> constant_vector(input_ctrl->get_state_input_port().size());
  // constant_vector.setZero();
  // auto constant_zero_source =
  //     base_builder->AddSystem<ConstantVectorSource<double>>(constant_vector);
  // base_builder->Connect(constant_zero_source->get_output_port(),
  //                       input_ctrl->get_state_input_port());

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);

  Context<double> &plant_context =
      sys->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());
  // Eigen::VectorXd x = CassieFixedPointState();
  // x[26] = 0.1;
  Eigen::VectorXd x(45);
  x << 0.0, 0.0, 0.8524750471115112,
      -0.2804698050022125, -0.0176906269043684, -0.009361587464809418,
      -0.9596542119979858, 0.02399946004152298, -0.035354699939489365,
      -0.003467952599748969, 0.020623575896024704, 0.7730164527893066,
      0.7827988862991333, -1.6299140453338623, -1.6473208665847778,
      -0.004993573296815157, -0.004239166621118784, 1.8716217279434204,
      1.8911134004592896, -0.011460183188319206, -0.014697963371872902,
      -1.9053080081939697, -1.909773349761963,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  plant->set_state_vector(&plant_context, x);

  simulator.Initialize();
  simulator.StepTo(5.0);
  // simulator.StepTo(std::numeric_limits<double>::infinity());

  while (true) {
    viz->ReplayCachedSimulation();
  }

  return 0;
}

}  // namespace
}  // namespace cassie

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Choose \"sensor\" or \"state\" to visualize" << std::endl;
    return 1;
  }
  std::string sim_type = (std::string) argv[1];
  if (sim_type != "sensor" && sim_type != "state") {
    std::cout << "Choose \"sensor\" or \"state\" to visualize" << std::endl;
    return 1;
  }
  return cassie::doMain(sim_type);
}
