/// Implements a visualizer of Cassie's State
/// Can visualize from sensor readings or estimated state

#include "cassie/cassie_common.h"
#include "cassie/comms/cassie_lcm_parsing.h"
#include "cassie_types/lcmt_cassie_sensor.hpp"
#include "cassie_types/lcmt_cassie_state.hpp"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"

#include <lcm/lcm-cpp.hpp>

using drake::manipulation::SimpleTreeVisualizer;

namespace cassie {
namespace {

class OutputVisualizer {
 private:
  drake::lcm::DrakeLcm *lcmViz;
  std::unique_ptr<RigidBodyTree<double>> tree;
  SimpleTreeVisualizer *cassieVisualizer;
  CassiePosition x;

 public:
  explicit OutputVisualizer(drake::lcm::DrakeLcm *lcm) {
    lcmViz = lcm;
    tree = getCassieTreed();
    cassieVisualizer = new SimpleTreeVisualizer(*tree.get(), lcmViz);

    x = CassiePosition::Zero();
    x[2] = 1.1;
    x[3] = 1;
    cassieVisualizer->visualize(x);
  }

  ~OutputVisualizer() {}

  void sensorHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const cassie_types::lcmt_cassie_sensor *sensor_msg) {
    Vector4<double> q;

    for (int ii = 0; ii < 4; ii++)
      q[ii] = static_cast<double>(sensor_msg->imu_orientation[ii]);

    x.segment(3, 4) = q;

    for (int ii = 0; ii < 10; ii++)
      x[kCassieMotorToState[ii]] =
          static_cast<double>(sensor_msg->motor_positions[ii]);
    for (int ii = 0; ii < 4; ii++)
      x[kCassieJointToState[ii]] =
          static_cast<double>(sensor_msg->joint_positions[ii]);

    cassieVisualizer->visualize(x);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  void stateHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                    const cassie_types::lcmt_cassie_state *state_msg) {
    for (int ii = 0; ii < kCassiePositions; ii++)
      x[ii] = static_cast<double>(state_msg->q[ii]);

    cassieVisualizer->visualize(x);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
};

int DoMain(std::string vizType) {
  drake::lcm::DrakeLcm lcm;
  lcm::LCM *lc = lcm.get_lcm_instance();
  lcm::Subscription *sub;

  if (!lc->good())
    return 1;

  OutputVisualizer lcmHandler(&lcm);

  if (vizType == "sensor") {
    sub = lc->subscribe("CASSIE_SENSOR", &OutputVisualizer::sensorHandler,
                        &lcmHandler);
    sub->setQueueCapacity(1);
  } else if (vizType == "state") {
    sub = lc->subscribe("CASSIE_STATE", &OutputVisualizer::stateHandler,
                        &lcmHandler);
    sub->setQueueCapacity(1);
  } else {
    std::cout << "Choose \"sensor\" or \"state\" to visualize" << std::endl;
    return 1;
  }

  while (true) {
    lc->handle();
  }

  return 0;
}

}  // namespace
}  // namespace cassie

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cout << "Choose \"sensor\" or \"state\" to visualize" << std::endl;
    return 1;
  }
  return cassie::DoMain((std::string)argv[1]);
}
