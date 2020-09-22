#include <iostream>
#include <memory>
#include <string>

#include <drake/systems/controllers/pid_controlled_system.h>
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time (usually between 0 and 1). "
              "This is documented in Simulator::set_target_realtime_rate().");
DEFINE_double(simulation_time, 60.0, "Simulation duration in seconds");
DEFINE_double(
    time_step, 1.0E-4,
    "The fixed-time step period (in seconds) of discrete updates for the "
    "multibody plant modeled as a discrete system. Strictly positive.");
DEFINE_double(penetration_allowance, 1.0E-3, "Allowable penetration (meters).");
DEFINE_double(stiction_tolerance, 1.0E-3,
              "Allowable drift speed during stiction (m/s).");
DEFINE_double(Kp_, 10.0, "p");
DEFINE_double(Ki_, 0.0, "i");
DEFINE_double(Kd_, 0.0, "d");

namespace drake {
namespace examples {
namespace a1 {
namespace {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using Eigen::Translation3d;
using Eigen::VectorXd;
using std::cout;
using std::endl;

int do_main() {
  if (FLAGS_time_step <= 0) {
    throw std::runtime_error(
        "time_step must be a strictly positive number. Only the time-stepping "
        "mode is supported for this model.");
  }

  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto pair = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));
  MultibodyPlant<double>& plant = pair.plant;

  const std::string full_name =
      FindResourceOrThrow("drake/examples/a1/A1_description/urdf/a1.urdf");
  multibody::Parser(&plant).AddModelFromFile(full_name);

  // Add model of the ground.
  const double static_friction = 1.0;
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
  plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(),
                               geometry::HalfSpace(), "GroundVisualGeometry",
                               green);
  // For a time-stepping model only static friction is used.
  const multibody::CoulombFriction<double> ground_friction(static_friction,
                                                           static_friction);
  plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(),
                                  geometry::HalfSpace(),
                                  "GroundCollisionGeometry", ground_friction);

  plant.Finalize();
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

  //   Create PID Controller.
//   const Eigen::VectorXd Kp =
//       Eigen::VectorXd::Ones(plant.num_positions()) * FLAGS_Kp_;
//   const Eigen::VectorXd Ki =
//       Eigen::VectorXd::Ones(plant.num_positions()) * FLAGS_Ki_;
//   const Eigen::VectorXd Kd =
//       Eigen::VectorXd::Ones(plant.num_positions()) * FLAGS_Kd_;
//   const auto* const pid =
//       builder.AddSystem<systems::controllers::PidController<double>>(Kp, Ki,
//                                                                      Kd);
//   builder.Connect(plant.get_state_output_port(),
//                   pid->get_input_port_estimated_state());
//   builder.Connect(pid->get_output_port_control(),
//                   plant.get_actuation_input_port());

  const drake::multibody::Body<double>& a1_base = plant.GetBodyByName("base");
  ConnectContactResultsToDrakeVisualizer(&builder, plant);
  geometry::ConnectDrakeVisualizer(&builder, pair.scene_graph);
  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  for (int i = 0; i < plant.num_input_ports(); i++) {
    cout << "Input Port [" << i << "] : " << plant.get_input_port(i).get_name()
         << endl;
  }
  cout << endl;
  for (int i = 0; i < plant.num_output_ports(); i++) {
    cout << "Output Port [" << i
         << "] : " << plant.get_output_port(i).get_name() << endl;
  }

  VectorXd tau = VectorXd::Zero(plant.num_actuated_dofs()) * 2;
    for (int i = 0; i < plant.num_actuated_dofs(); i++) {
      if (i % 3 != 0) {
        tau[i] = 6;
      } else {
        tau[i] = i % 6 == 0 ? -1 : 1;  // legs outward
      }
    }
  plant.get_actuation_input_port().FixValue(&plant_context, tau);

  // Set the a1_base frame P initial pose.
  const Translation3d X_WP(0.0, 0.0, 0.5);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, a1_base, X_WP);

  auto simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));
  simulator->AdvanceTo(1);




  const systems::OutputPort<double>& output = plant.get_state_output_port();
  cout << "Output port [" << output.get_name() << "] of size: ["
       << output.Eval(plant_context).size() << "] : " << output.get_name()
       << endl;
  for (int i = 0; i < output.Eval(plant_context).size(); i++) {
    cout << output.Eval(plant_context)[i] << " ";
  }
  cout << endl;

  const Eigen::VectorBlock<const Eigen::VectorXd> vec =
      plant.GetPositions(plant_context);
  cout << "positions are : [" << vec.size() << "] : " << endl;
  for (int i = 0; i < vec.size(); i++) {
    cout << vec[i] << " ";
  }
  cout << endl;



  simulator->AdvanceTo(FLAGS_simulation_time);
  return 0;
}

}  // namespace
}  // namespace a1
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "\nPassive simulation of the Atlas robot. With the default time step of "
      "\n1 ms this simulation typically runs slightly faster than real-time. "
      "\nThe time step has an effect on the joint limits stiffnesses, which "
      "\nconverge quadratically to the rigid limit as the time step is "
      "\ndecreased. Thus, decrease the time step for more accurately resolved "
      "\njoint limits. "
      "\nLaunch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::a1::do_main();
}
