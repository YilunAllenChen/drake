#include "cassie/util/csv_parser.h"
#include "cassie_types/lcmt_cassie_sensor.hpp"

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <unistd.h>

using Eigen::MatrixXf;

// run with:
// bazel run //cassie/filter:cassie_log_streamer
// /cassie/cassie/filter/data/2017-10-25/outputs_trimmed_86_105.csv

void initialize_message(cassie_types::lcmt_cassie_sensor &msg) {
  msg.utime = 0;

  // joints
  for (int i = 0; i < 4; i++) {
    msg.joint_positions[i] = 0;
    msg.joint_velocities[i] = 0;
    msg.spring_deflections[i] = 0;
    msg.spring_velocities[i] = 0;
  }

  // motors
  for (int i = 0; i < 10; i++) {
    msg.motor_positions[i] = 0;
    msg.motor_velocities[i] = 0;
    msg.measured_torques[i] = 0;
  }

  // 3-axis sensors
  for (int i = 0; i < 3; i++) {
    msg.imu_angular_velocity[i] = 0;
    msg.imu_linear_acceleration[i] = 0;
    msg.imu_orientation[i + 1] = 0;
    msg.imu_magnetic_field[i] = 0;
  }
  msg.imu_orientation[0] = 1;
}

void log_to_message(cassie_types::lcmt_cassie_sensor &msg,
                    const Eigen::VectorXf &data) {
  initialize_message(msg);

  msg.utime = static_cast<int>(data.tail(1)[0] * 1e6);
  Eigen::Map<Eigen::Vector4f>(msg.imu_orientation, 1, 4) = data.segment(0, 4);
  Eigen::Map<Eigen::Vector3f>(msg.imu_angular_velocity, 1, 3) =
      data.segment(4, 3);
  Eigen::Map<Eigen::Matrix<float, 1, 10>>(msg.motor_positions, 1, 10) =
      data.segment(7, 10);
  Eigen::Map<Eigen::Matrix<float, 1, 10>>(msg.motor_velocities, 1, 10) =
      data.segment(17, 10);
  Eigen::Map<Eigen::Vector4f>(msg.joint_positions, 1, 4) = data.segment(27, 4);
  Eigen::Map<Eigen::Vector4f>(msg.joint_velocities, 1, 4) = data.segment(31, 4);
}

int main(int argc, char *argv[]) {
  if (argc <= 1) {
    std::cout << "no file given" << std::endl;
    return 1;
  }

  lcm::LCM lcm;
  if (!lcm.good())
    return 1;

  // read the log file
  MatrixXf data = load_csv<MatrixXf>(argv[1]);

  // send log data as LCM messgaes
  cassie_types::lcmt_cassie_sensor msg;
  for (int i = 0; i < data.rows(); i++) {
    log_to_message(msg, data.row(i));
    lcm.publish("CASSIE_SENSOR", &msg);

    // pause for the appropriate amount of time between messages
    usleep(500);
  }

  return 0;
}
