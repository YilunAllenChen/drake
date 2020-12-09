#include "UDPDriver.hpp"

#include <string>

// Start Position of each part of published LCM in UDP message
// Measured after header and utime
#define IN_IMU_ORIENT 0
#define IN_IMU_ANG_VEL 4
#define IN_IMU_LIN_ACC 7
#define IN_IMU_MAG 10
#define IN_MOTOR_POS 13
#define IN_MOTOR_VEL 23
#define IN_TORQUE 33
#define IN_JOINT_POS 43
#define IN_JOINT_VEL 47
// #define IN_SPRING_POS 51
// #define IN_SPRING_VEL 55
#define IN_RADIO 51
#define IN_SOC 67
// #define IN_STATUS 68
#define IN_PRESS 68
#define IN_TEMP 69

// Start Position of each part of received LCM in UDP message
// Measured after header and utime
#define OUT_FF_TORQUE 0
#define OUT_Q_D 10
#define OUT_QDOT_D 20
#define OUT_KP 30
#define OUT_KD 40
#define OUT_KI 50
#define OUT_LEAK 60
#define OUT_CLAMP 70

/////////////////// CONSTRUCTOR ////////////////////////////
UDPDriver::UDPDriver(boost::shared_ptr<lcm::LCM> &lcm) : lcm_(lcm) {}

UDPDriver::~UDPDriver() {}

/////////////////// PUBLISH ////////////////////////////////
void UDPDriver::publishSensor(unsigned char *buf) {
  cassie_types::lcmt_cassie_sensor msg_sensor;
  // float readVal;
  buf += UDP_HEADER_SIZE;

  memcpy(&msg_sensor.utime, &buf[0], UTIME_SIZE);
  buf += UTIME_SIZE;

  for (int ii = 0; ii < 3; ii++) {
    memcpy(&msg_sensor.imu_angular_velocity[ii],
           &buf[(IN_IMU_ANG_VEL + ii) * NUMSIZE], NUMSIZE);
    memcpy(&msg_sensor.imu_linear_acceleration[ii],
           &buf[(IN_IMU_LIN_ACC + ii) * NUMSIZE], NUMSIZE);
    memcpy(&msg_sensor.imu_magnetic_field[ii],
           &buf[(IN_IMU_MAG + ii) * NUMSIZE], NUMSIZE);
  }

  for (int ii = 0; ii < 4; ii++) {
    memcpy(&msg_sensor.imu_orientation[ii],
           &buf[(IN_IMU_ORIENT + ii) * NUMSIZE], NUMSIZE);
    memcpy(&msg_sensor.joint_positions[ii],
           &buf[(IN_JOINT_POS + ii) * NUMSIZE], NUMSIZE);
    memcpy(&msg_sensor.joint_velocities[ii],
           &buf[(IN_JOINT_VEL + ii) * NUMSIZE], NUMSIZE);
    // memcpy(&msg_sensor.spring_deflections[ii],
    //        &buf[(IN_SPRING_POS + ii) * NUMSIZE], NUMSIZE);
    // memcpy(&msg_sensor.spring_velocities[ii],
    //        &buf[(IN_SPRING_VEL + ii) * NUMSIZE], NUMSIZE);
    msg_sensor.spring_deflections[ii] = 0;
    msg_sensor.spring_velocities[ii] = 0;
  }

  for (int ii = 0; ii < 10; ii++) {
    memcpy(&msg_sensor.motor_positions[ii],
           &buf[(IN_MOTOR_POS + ii) * NUMSIZE], NUMSIZE);
    memcpy(&msg_sensor.motor_velocities[ii],
           &buf[(IN_MOTOR_VEL + ii) * NUMSIZE], NUMSIZE);
    memcpy(&msg_sensor.measured_torques[ii],
           &buf[(IN_TORQUE + ii) * NUMSIZE], NUMSIZE);
  }

  lcm_->publish(("CASSIE_SENSOR"), &msg_sensor);
}

void UDPDriver::publishStatus(unsigned char *buf) {
  cassie_types::lcmt_cassie_status msg_status;
  float readVal;
  buf += UDP_HEADER_SIZE;

  memcpy(&msg_status.utime, &buf[0], UTIME_SIZE);
  buf += UTIME_SIZE;

  memcpy(&readVal, &buf[IN_SOC * NUMSIZE], NUMSIZE);
  msg_status.charge_state = static_cast<int32_t>(readVal * 100);

  // memcpy(&readVal, &buf[IN_STATUS * NUMSIZE], NUMSIZE);
  // msg_status.status = static_cast<int32_t>(readVal);
  msg_status.status = 0;

  memcpy(&msg_status.temperature, &buf[IN_TEMP * NUMSIZE], NUMSIZE);

  memcpy(&msg_status.pressure, &buf[IN_PRESS * NUMSIZE], NUMSIZE);

  for (int ii = 0; ii < 16; ii++) {
    memcpy(&msg_status.radio[ii], &buf[(IN_RADIO + ii) * NUMSIZE], NUMSIZE);
  }

  lcm_->publish(("CASSIE_STATUS"), &msg_status);
}

/////////////////// HANDLER ////////////////////////////////
void UDPDriver::commandHandler(const lcm::ReceiveBuffer *rbuf,
                               const std::string &channel,
                               const cassie_types::lcmt_cassie_command *msg) {
  int offset = UDP_HEADER_SIZE + UTIME_SIZE;

  memcpy(&dataOut[UDP_HEADER_SIZE], &msg->utime, UTIME_SIZE);

  for (int ii = 0; ii < 10; ii++) {
    memcpy(&dataOut[(OUT_FF_TORQUE + ii) * NUMSIZE + offset],
           &msg->motor_torques[ii], NUMSIZE);

    memcpy(&dataOut[(OUT_Q_D + ii) * NUMSIZE + offset],
           &msg->motor_positions_desired[ii], NUMSIZE);

    memcpy(&dataOut[(OUT_QDOT_D + ii) * NUMSIZE + offset],
           &msg->motor_velocities_desired[ii], NUMSIZE);

    memcpy(&dataOut[(OUT_KP + ii) * NUMSIZE + offset],
           &msg->motor_kp[ii], NUMSIZE);

    memcpy(&dataOut[(OUT_KD + ii) * NUMSIZE + offset],
           &msg->motor_kd[ii], NUMSIZE);

    memcpy(&dataOut[(OUT_KI + ii) * NUMSIZE + offset],
           &msg->motor_ki[ii], NUMSIZE);

    memcpy(&dataOut[(OUT_LEAK + ii) * NUMSIZE + offset],
           &msg->leak_factor[ii], NUMSIZE);

    memcpy(&dataOut[(OUT_CLAMP + ii) * NUMSIZE + offset],
           &msg->integrator_clamp[ii], NUMSIZE);
  }
}
