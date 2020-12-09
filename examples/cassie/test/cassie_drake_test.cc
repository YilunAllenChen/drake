#include "cassie/cassie_common.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include <gtest/gtest.h>

namespace cassie {
namespace {
using drake::MatrixCompareType;
using drake::manipulation::util::SimDiagramBuilder;
using drake::systems::ConstantVectorSource;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
using Eigen::MatrixXd;
using Eigen::VectorXd;

GTEST_TEST(CassieDrake, RigidBodyTree) {
  auto rtree = getCassieTreed();
  VectorXd x = CassieFixedPointState();
  VectorXd q = x.head(kCassiePositions);
  VectorXd v = x.tail(kCassieVelocities);

  std::vector<std::string> spring_name = {
    "knee_spring_left_fixed", "knee_spring_right_fixed",
    "ankle_spring_joint_left", "ankle_spring_joint_right"
  };
  int spring_num[4] = {15, 16, 19, 20};
  for (int ii = 0; ii < 4; ii++) {
    EXPECT_EQ(rtree->FindIndexOfChildBodyOfJoint(spring_name[ii]) + 3,
              spring_num[ii]);
  }

  EXPECT_EQ(rtree->FindBody("toe_left")->get_body_index(), 20);
  EXPECT_EQ(rtree->FindBody("toe_right")->get_body_index(), 21);

  auto kinsol = rtree->doKinematics(q, v);
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;

  MatrixXd H = rtree->massMatrix(kinsol);
  MatrixXd H_real(kCassieVelocities, kCassieVelocities);

  H_real << 2.361075147, 0.00067854, -0.37454428, 0, 3.82354625, 0.00638861, 0.90013367, 0.89981580, -0.09059959, -0.09059510, 0.00467267, -0.00522449, -0.04721353, 0.04646201, -0.04449751, 0.04378072, 0.01260820, -0.01244567, -0.00094392, 0.00094316, 0.00028626, -0.00028625,  // NOLINT
      0.00067854, 2.14626889, 0.00224868, -3.82354625, 0, 0.37268878, 0.00738897, -0.00669906, 0.01126844, -0.00943070, -0.78436588, -0.78436350, -0.60239531, -0.60239235, -0.49889623, -0.49889341, -0.10551866, -0.10551871, 0.00189436, 0.00189434, -0.00362458, -0.00362458,  // NOLINT
      -0.37454428, 0.00224868, 1.00939099, -0.00638861, -0.37268878, 0, -0.18735302, -0.18734965, 0.17306234, 0.17274169, -0.16663396, 0.16504937, -0.09656667, 0.09525455, -0.07125392, 0.07016028, -0.01556115, 0.01532001, 0.00151325, -0.00151147, -0.00047152, 0.00047152,  // NOLINT
      0, -3.82354625, -0.00638861, 32.768, 0, 0, 0, 0, -0.18528483, 0.18247039, 1.27121142, 1.27121103, 0.74443797, 0.74443757, 0.55021160, 0.55021121, 0.12017446, 0.12017407, -0.00905394, -0.00905394, 0.00361157, 0.00361157,  // NOLINT
      3.82354625, 0, -0.37268878, 0, 32.768, 0, 2.05849904, 2.05852206,-0.05825477, -0.05824584, 0.00368693, -0.00368610, 0.00215911, -0.00215863, 0.00159579, -0.00159543, 0.00034854, -0.00034846, -0.00082059, 0.00082059, 1.0474e-05, -1.0472e-05,  // NOLINT
      0.00638861, 0.37268878, 0, 0, 0, 32.768, 0.18546683, -0.18265239, 0, 0, 0.00753097, 0.00753162, -0.37866915, -0.37866850, -0.35538142, -0.35538077, 0.09433588, 0.09433652, -0.00462078, -0.00462078, 0.00209909, 0.00209909,  // NOLINT
      0.90013367, 0.00738897, -0.18735302, 0, 2.05849904, 0.18546683, 0.87509557, 0, -0.09059954, 0, 0.00365599, 0, 0.00390680, 0, 0.00347897, 0, -0.00012713, 0, -0.00032011, 0, 2.885e-06, 0,  // NOLINT
      0.89981580, -0.00669906, -0.18734965, 0, 2.05852206, -0.18265239, 0, 0.87515766, 0, -0.09059505, 0, -0.00420772, 0, -0.00465823, 0, -0.00419567, 0, 0.00028975, 0, 0.00031936, 0, -2.882e-06,  // NOLINT
      -0.09059959, 0.01126844, 0.17306234, -0.18528483, -0.05825477, 0, -0.09059954, 0, 0.14519440, 0, 0.00516023, 0, 0.00403824, 0, 0.00310283, 0, 0.00067948, 0, 0.00025076, 0, 1.654e-05, 0,  // NOLINT
      -0.09059510, -0.00943070, 0.17274169, 0.18247039, -0.05824584, 0, 0, -0.09059505, 0, 0.14525413, 0, -0.00674473, 0, -0.00535028, 0, -0.00419640, 0, -0.00092056, 0, -0.00024898, 0, -1.654e-05,  // NOLINT
      0.00467267, -0.78436588, -0.16663396, 1.27121142, 0.00368693, 0.00753097, 0.00365599, 0, 0.00516023, 0, 0.67033266, 0, 0.51684968, 0, 0.43197134, 0, 0.09932454, 0, -0.00130663, 0, 0.00340238, 0,  // NOLINT
      -0.00522449, -0.78436350, 0.16504937, 1.27121103, -0.00368610, 0.00753162, 0, -0.00420772, 0, -0.00674473, 0, 0.67033195, 0, 0.51684897, 0, 0.43197067, 0, 0.09932418, 0, -0.00130661, 0, 0.00340238,  // NOLINT
      -0.04721353, -0.60239531, -0.09656667, 0.74443797, 0.00215911, -0.37866915, 0.00390680, 0, 0.00403824, 0, 0.51684968, 0, 0.47163383, 0, 0.40390680, 0, 0.08100129, 0, -0.00010225, 0, 0.00290391, 0,  // NOLINT
      0.04646201, -0.60239235, 0.09525455,  0.744437579, -0.00215863, -0.37866850, 0, -0.00465823, 0, -0.00535028, 0, 0.51684897, 0, 0.47163311, 0, 0.40390611, 0, 0.08100092, 0, -0.00010223, 0, 0.00290391,  // NOLINT
      -0.04449751, -0.49889623, -0.07125392, 0.55021160, 0.00159579, -0.35538142, 0.00347897, 0, 0.00310283, 0, 0.43197134, 0, 0.40390680, 0, 0.35808774, 0, 0.07286977, 0, 0.00053767, 0, 0.00265150, 0,  // NOLINT
      0.04378072, -0.49889341, 0.07016028, 0.55021121, -0.00159543, -0.35538077, 0, -0.00419567, 0, -0.00419640, 0, 0.43197067, 0, 0.40390611, 0, 0.35808709, 0, 0.07286945, 0, 0.00053769, 0, 0.00265150,  // NOLINT
      0.01260820, -0.10551866, -0.01556115, 0.12017446, 0.00034854, 0.09433588, -0.00012713, 0, 0.00067948, 0, 0.09932454, 0, 0.08100129, 0, 0.07286977, 0, 0.06595072, 0, 0.00183673, 0, 0.00221356, 0,  // NOLINT
      -0.01244567, -0.10551871, 0.01532001, 0.12017407, -0.00034846, 0.09433652, 0, 0.00028975, 0, -0.00092056, 0, 0.09932418, 0, 0.08100092, 0, 0.07286945, 0, 0.06595071, 0, 0.00183675, 0, 0.00221356,  // NOLINT
      -0.00094392, 0.00189436, 0.00151325, -0.00905394, -0.00082059, -0.00462078, -0.00032011, 0, 0.00025076, 0, -0.00130663, 0, -0.00010225, 0, 0.00053767, 0,  0.00183673, 0, 0.00185404, 0, 0, 0,  // NOLINT
      0.00094316, 0.00189434, -0.00151147, -0.00905394, 0.00082059, -0.00462078, 0, 0.00031936, 0, -0.00024898, 0, -0.00130661, 0, -0.00010223, 0, 0.00053769, 0, 0.00183675, 0, 0.00185404, 0, 0,  // NOLINT
      0.00028626, -0.00362458, -0.00047152, 0.00361157, 1.047e-05, 0.00209909, 2.885e-06, 0, 1.654e-05, 0, 0.00340238, 0, 0.00290391, 0, 0.00265150, 0, 0.00221356, 0, 0, 0, 0.00056558, 0,  // NOLINT
      -0.00028625, -0.00362458, 0.00047152, 0.00361157, -1.047e-05, 0.00209909, 0, -2.882e-06, 0, -1.654e-05, 0, 0.00340238, 0, 0.00290391, 0, 0.00265150, 0, 0.00221356, 0, 0, 0, 0.00056558;  // NOLINT

  EXPECT_TRUE(CompareMatrices(H, H_real, 1e-7, MatrixCompareType::absolute));

  VectorXd C = rtree->dynamicsBiasTerm(kinsol, no_external_wrenches, true);
  VectorXd C_real(kCassieVelocities);
  C_real << 0.062672318, 3.656076968, 0, 0, 0, 321.454080000, 1.819429649,
      -1.791820037, 0, 0, 0.073878907, 0.073885250, -3.714744367, -3.714738023,
      -65.886291751, -65.886285408, 0.925435005, 0.925441348, -40.670329897,
      -40.670329897, 0.020592085, 0.020592085;
  EXPECT_TRUE(CompareMatrices(C, C_real, 1e-8, MatrixCompareType::absolute));

  MatrixXd B = rtree->B;
  MatrixXd B_real = MatrixXd::Zero(kCassieVelocities, kCassieActuators);
  B_real.block<8, 8>(6, 0) = MatrixXd::Identity(8, 8);
  B_real.block<2, 2>(20, 8) = MatrixXd::Identity(2, 2);
  EXPECT_TRUE(CompareMatrices(B, B_real, 1e-8, MatrixCompareType::absolute));

  VectorXd com = rtree->centerOfMass(kinsol);
  VectorXd com_real(3); com_real << -0.011373559, 0.000194965, 0.790814628;
  EXPECT_TRUE(CompareMatrices(com, com_real, 1e-8,
                              MatrixCompareType::absolute));

  MatrixXd Jcom = rtree->centerOfMassJacobian(kinsol);
  MatrixXd Jcom_real(3, kCassieVelocities);
  Jcom_real << 0, -0.11668537, -0.00019496, 1, 0, 0, 0, 0, -0.00565444, 0.00556855, 0.03879429, 0.03879428, 0.02271844, 0.02271843, 0.01679112, 0.01679111, 0.00366743, 0.00366742, -0.00027630, -0.00027630, 0.00011021, 0.00011021,  // NOLINT
      0.11668537, 0, -0.01137355, 0, 1, 0, 0.06282040, 0.06282110, -0.00177779, -0.00177752, 0.00011251, -0.00011249, 6.5891e-05, -6.5876e-05, 4.8699e-05, -4.8688e-05, 1.0636e-05, -1.0634e-05, -2.5042e-05, 2.5042e-05, 3.1966e-07, -3.1959e-07,  // NOLINT
      0.00019496, 0.01137355, 0, 0, 0, 1, 0.00565999, -0.00557410, 0, 0, 0.00022982, 0.00022984, -0.01155606, -0.01155604, -0.01084538, -0.01084536, 0.00287890, 0.00287892, -0.00014101, -0.00014101, 6.4059e-05, 6.4059e-05;  // NOLINT
  EXPECT_TRUE(CompareMatrices(Jcom, Jcom_real, 1e-8,
                              MatrixCompareType::absolute));

  VectorXd phi = rtree->positionConstraints(kinsol);
  VectorXd phi_real = VectorXd::Constant(2, -7.28855e-06);
  EXPECT_TRUE(CompareMatrices(phi, phi_real, 1e-8,
                              MatrixCompareType::absolute));

  MatrixXd Jphi = rtree->positionConstraintsJacobian(kinsol, false);
  MatrixXd Jphi_real = MatrixXd::Zero(2, kCassieVelocities);
  Jphi_real.block<2, 8>(0, 12) << 0.11633612, 0, 0.15726796, 0, 0.11556318, 0, 0.10283175, 0,  // NOLINT
                                  0, 0.11633612, 0, 0.15726796, 0, 0.11556318, 0, 0.10283175;  // NOLINT
  EXPECT_TRUE(CompareMatrices(Jphi, Jphi_real, 1e-8,
                              MatrixCompareType::absolute));
}

GTEST_TEST(CassieDrake, DiscreteSim) {
  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;

  auto tree = getCassieTreed();
  auto plantDt =
      std::make_unique<RigidBodyPlant<double>>(std::move(tree), 0.0005);
  RigidBodyPlant<double>* plant = builder.AddPlant(std::move(plantDt));

  builder.AddVisualizer(&lcm);
  builder.get_visualizer()->set_publish_period(1e-2);

  setDefaultContactParams(*plant);

  drake::systems::DiagramBuilder<double> *base_builder =
      builder.get_mutable_builder();

  VectorX<double> constant_vector(plant->get_input_port(0).size());
  constant_vector.setZero();
  auto constant_zero_source =
      base_builder->AddSystem<ConstantVectorSource<double>>(constant_vector);
  constant_zero_source->set_name("zero input");

  base_builder->Connect(constant_zero_source->get_output_port(),
                  plant->get_input_port(0));

  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(1.0);

  plant->set_state_vector(&simulator.get_mutable_context(),
                          CassieFixedPointState());

  const Context<double> *plant_context =
      &sys->GetSubsystemContext(*plant, simulator.get_context());

  simulator.Initialize();
  VectorX<double> start_state = plant->GetStateVector(*plant_context);
  simulator.StepTo(0.4);
  VectorX<double> end_state = plant->GetStateVector(*plant_context);

  VectorX<double> end_state_real(kCassieStates);
  end_state_real << 0.000860446, 0.000328502, 0.367273136,
      0.948110075, 0.001586788, -0.318923477, 0.005102038,
      0.026483902, -0.038901160, 0.044302147, -0.059001229,
      0.518651023, 0.500334504, -2.766099762, -2.765618180,
      -0.002663651, -0.002863886, 2.998307820, 2.998476438,
      -0.005829680, -0.006050671, -2.271051658, -2.253874791,
      0.017718798, -7.355500578, 0.178260109,
      0.185749979, 0.000198601, -0.114367152,
      0.550126684, -0.655845167, -0.021314350, -0.136274840,
      -11.853061244, -12.325629739, 0.096232461, 0.450394931,
      0.036463195, -0.056315298, -0.151758207, -0.218700139,
      -0.002984077, -0.095021149, 4.461506734, 4.551464113;

  EXPECT_TRUE(CompareMatrices(start_state, CassieFixedPointState(), 1e-8,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(end_state, end_state_real, 1e-8,
                              MatrixCompareType::absolute));
}

GTEST_TEST(CassieDrake, ContinuousSim) {
  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;

  auto tree = getCassieTreed();
  auto plantDt =
      std::make_unique<RigidBodyPlant<double>>(std::move(tree));
  RigidBodyPlant<double>* plant = builder.AddPlant(std::move(plantDt));

  builder.AddVisualizer(&lcm);
  builder.get_visualizer()->set_publish_period(1e-2);

  setDefaultContactParams(*plant);

  drake::systems::DiagramBuilder<double> *base_builder =
      builder.get_mutable_builder();

  VectorX<double> constant_vector(plant->get_input_port(0).size());
  constant_vector.setZero();
  auto constant_zero_source =
      base_builder->AddSystem<ConstantVectorSource<double>>(constant_vector);
  constant_zero_source->set_name("zero input");

  base_builder->Connect(constant_zero_source->get_output_port(),
                  plant->get_input_port(0));

  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(1.0);

  plant->set_state_vector(&simulator.get_mutable_context(),
                          CassieFixedPointState());

  const Context<double> *plant_context =
      &sys->GetSubsystemContext(*plant, simulator.get_context());

  simulator.Initialize();
  VectorX<double> start_state = plant->GetStateVector(*plant_context);
  simulator.StepTo(0.4);
  VectorX<double> end_state = plant->GetStateVector(*plant_context);

  VectorX<double> end_state_real(kCassieStates);
  end_state_real << -0.00665892, 0.00016062, 0.17450134, 0.98561767,
      -0.00013243, -0.16899050, -1.8894e-05, 0.01648634, -0.01876915,
      0.04366675, -0.04318442, 1.43908638, 1.43874963, -3.04912427,
      -3.04909067, -0.03862475, -0.03860100, 3.40160823, 3.40145822,
      -0.01371231, -0.01371026, -2.55680354, -2.55679576, -0.00920320,
      -1.26114832, -0.00084153, -0.12188371, 0.00412540, -1.16319148,
      0.27986639, -0.32815202, 0.79538941, -0.77894250, -0.45910591,
      -0.46344045, -1.42328852, -1.42088039, -0.04209248, -0.04211459,
      2.84932979, 2.844204370, -0.18228290, -0.18244540, -0.71805693,
      -0.71493524;

  EXPECT_TRUE(CompareMatrices(start_state, CassieFixedPointState(), 1e-8,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(end_state, end_state_real, 1e-8,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace cassie
