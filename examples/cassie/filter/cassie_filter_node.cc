#include "cassie/cassie_common.h"
#include "cassie/comms/cassie_lcm_parsing.h"
#include "cassie/filter/cassie_kinematic_filter.h"
#include "cassie_types/lcmt_cassie_sensor.hpp"
#include "cassie_types/lcmt_cassie_state.hpp"
#include "cassie_types/lcmt_flag.hpp"
#include "drake/multibody/rigid_body_tree.h"

#include <lcm/lcm-cpp.hpp>

namespace cassie {
namespace filter {

using cassie_types::lcmt_cassie_state;
using cassie_types::lcmt_cassie_sensor;
using cassie_types::lcmt_flag;

lcm::LCM lcm_proc;

CassieMeasurement
measurement_from_lcm_sensor(const cassie_types::lcmt_cassie_sensor *msg) {
  CassieMeasurement y;

  for (int i = 0; i < 4; i++) {
    y.joint_positions[kCassieJointToState[i] - 7] =
        static_cast<double>(msg->joint_positions[i]);
    y.joint_velocities[kCassieJointToState[i] - 7] =
        static_cast<double>(msg->joint_velocities[i]);
  }

  for (int i = 0; i < 10; i++) {
    y.joint_positions[kCassieMotorToState[i] - 7] =
        static_cast<double>(msg->motor_positions[i]);
    y.joint_velocities[kCassieMotorToState[i] - 7] =
        static_cast<double>(msg->motor_velocities[i]);
  }

  for (int i = 0; i < 4; i++) {
    if (kCassieSpringToState[i] >= 0) {
      y.joint_positions[kCassieSpringToState[i] - 7] =
          static_cast<double>(msg->spring_deflections[i]);
      y.joint_velocities[kCassieSpringToState[i] - 7] =
          static_cast<double>(msg->spring_velocities[i]);
    }
  }

  y.orientation =
      Eigen::Map<const Eigen::Matrix<float, 4, 1>>(msg->imu_orientation, 4, 1)
          .cast<double>();
  y.gyro = Eigen::Map<const Eigen::Matrix<float, 3, 1>>(
               msg->imu_angular_velocity, 3, 1)
               .cast<double>();
  y.accel = Eigen::Map<const Eigen::Matrix<float, 3, 1>>(
                msg->imu_linear_acceleration, 3, 1)
                .cast<double>();

  return y;
}

lcmt_cassie_state
lcm_state_from_cassie_state(int64_t utime, const CassieState &x,
                            const std::vector<bool> contact_mode) {
  lcmt_cassie_state msg;
  msg.utime = utime;
  for (int i = 0; i < kCassiePositions; i++) {
    msg.q[i] = static_cast<float>(x[i]);
  }
  for (int i = 0; i < kCassieVelocities; i++) {
    msg.v[i] = static_cast<float>(x[23 + i]);
  }
  msg.left_foot = contact_mode[0];
  msg.right_foot = contact_mode[1];
  return msg;
}

bool validate_measurement(const cassie_types::lcmt_cassie_sensor *msg) {
  // validate quaternion
  for (int i = 0; i < 4; i++) {
    if (msg->imu_orientation[i] != 0.0)
      return true;
  }
  return false;
}

class CassieSensorHandler {
 private:
  std::unique_ptr<CassieKinematicFilter> filter;

 public:
  CassieSensorHandler() {
    // filter parameters
    JointFilterParams joint_filter_params;
    ContactTriggerParams contact_trigger_params;
    KinematicFilterParams kinematic_filter_params;

    /*
    This is where you would edit filter parameters
    */

    // create filter object
    auto tree = getCassieTree<double>();
    auto cache = tree->CreateKinematicsCache();
    filter = std::make_unique<CassieKinematicFilter>(
        std::move(tree), cache, joint_filter_params, contact_trigger_params,
        kinematic_filter_params);
  }

  ~CassieSensorHandler() {}

  void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const cassie_types::lcmt_cassie_sensor *sensor_msg) {
    // test that message is valid
    if (!validate_measurement(sensor_msg)) {
      return;
    }

    CassieMeasurement y = measurement_from_lcm_sensor(sensor_msg);
    filter->update(5.0e-4, y);

    auto state_msg = lcm_state_from_cassie_state(
        sensor_msg->utime, filter->get_full_state(), filter->get_contact());
    lcm_proc.publish("CASSIE_STATE", &state_msg);
  }

  void handleReset(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                   const cassie_types::lcmt_flag *msg) {
    filter->reset();
  }
};

int DoMain() {
  if (!lcm_proc.good())
    return 1;

  CassieSensorHandler sensor_handler;

  lcm_proc.subscribe("CASSIE_SENSOR", &CassieSensorHandler::handleMessage,
                     &sensor_handler);
  lcm_proc.subscribe("CASSIE_RESET_ESTIMATE", &CassieSensorHandler::handleReset,
                     &sensor_handler);

  while (0 == lcm_proc.handle()) {}

  return 0;
}

}  // namespace filter
}  // namespace cassie

int main() { return cassie::filter::DoMain(); }
