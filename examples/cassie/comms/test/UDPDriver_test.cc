#include "cassie/comms/UDPDriver.hpp"
#include "cassie_types/lcmt_cassie_command.hpp"
#include "cassie_types/lcmt_cassie_sensor.hpp"
#include "cassie_types/lcmt_cassie_status.hpp"
#include <boost/shared_ptr.hpp>

#include <gtest/gtest.h>
#include <lcm/lcm-cpp.hpp>

namespace cassie {
namespace comms {
namespace {

void checkSensor(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                 const cassie_types::lcmt_cassie_sensor* msg, void*) {
    EXPECT_EQ(msg->utime, 1234);
    for (int ii = 0; ii < 3; ii++) {
        EXPECT_EQ(msg->imu_angular_velocity[ii], ii + 4);
        EXPECT_EQ(msg->imu_linear_acceleration[ii], ii + 7);
        EXPECT_EQ(msg->imu_magnetic_field[ii], ii + 10);
    }
    for (int ii = 0; ii < 4; ii++) {
        EXPECT_EQ(msg->joint_positions[ii], ii + 43);
        EXPECT_EQ(msg->joint_velocities[ii], ii + 47);
        EXPECT_EQ(msg->imu_orientation[ii], ii);
        // EXPECT_EQ(msg->spring_deflections[ii], ii + 71);
        // EXPECT_EQ(msg->spring_velocities[ii], ii + 75);
    }
    for (int ii = 0; ii < 10; ii++) {
        EXPECT_EQ(msg->motor_positions[ii], ii + 13);
        EXPECT_EQ(msg->motor_velocities[ii], ii + 23);
        EXPECT_EQ(msg->measured_torques[ii], ii + 33);
    }
}

void checkStatus(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                 const cassie_types::lcmt_cassie_status* msg, void*) {
    EXPECT_EQ(msg->utime, 1234);
    // EXPECT_EQ(msg->status, 60);
    EXPECT_EQ(msg->charge_state, 67*100);
    EXPECT_EQ(msg->pressure, 68);
    EXPECT_EQ(msg->temperature, 69);
    for (int ii = 0; ii < 16; ii++) {
        EXPECT_EQ(msg->radio[ii], ii + 51);
    }
}

void publishCommand(boost::shared_ptr<lcm::LCM> lcm) {
    cassie_types::lcmt_cassie_command msg;

    msg.utime = 0;
    for (int ii = 0; ii < 10; ii++) {
        msg.motor_torques[ii] = ii+0;
        msg.motor_positions_desired[ii] = ii+10;
        msg.motor_velocities_desired[ii] = ii+20;
        msg.motor_kp[ii] = ii+30;
        msg.motor_kd[ii] = ii+40;
        msg.motor_ki[ii] = ii+50;
        msg.leak_factor[ii] = ii+60;
        msg.integrator_clamp[ii] = ii+70;
    }

    lcm->publish(("CASSIE_COMMAND"), &msg);
}

GTEST_TEST(UDPDriver, PublishLcm) {
    boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
    UDPDriver driver(lcm);

    lcm->subscribeFunction("CASSIE_SENSOR", &checkSensor,
                           reinterpret_cast<void *>(NULL));
    lcm->subscribeFunction("CASSIE_STATUS", &checkStatus,
                           reinterpret_cast<void *>(NULL));

    unsigned char buf[DATA_IN_SIZE];
    int64_t utime = 1234;
    memcpy(&buf[UDP_HEADER_SIZE], &utime, UTIME_SIZE);
    for (int ii = 0;
         ii < (DATA_IN_SIZE - UDP_HEADER_SIZE - UTIME_SIZE) / NUMSIZE; ii++) {
      float val = ii;
      memcpy(&buf[ii * NUMSIZE + UDP_HEADER_SIZE + UTIME_SIZE], &val, NUMSIZE);
    }

    driver.publishSensor(buf);
    lcm_handle(driver.lcm_->getUnderlyingLCM());
    driver.publishStatus(buf);
    lcm_handle(driver.lcm_->getUnderlyingLCM());
}

GTEST_TEST(UDPDriver, HandleLcm) {
    boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
    UDPDriver driver(lcm);

    lcm->subscribe("CASSIE_COMMAND", &UDPDriver::commandHandler, &driver);

    publishCommand(lcm);
    lcm_handle(driver.lcm_->getUnderlyingLCM());

    int motor_vals = (DATA_OUT_SIZE - UDP_HEADER_SIZE - UTIME_SIZE)/NUMSIZE;
    // NOLINTNEXTLINE(runtime/arrays)
    float msg[motor_vals];
    memcpy(msg, &driver.dataOut[UDP_HEADER_SIZE + UTIME_SIZE], sizeof(msg));
    for (int ii = 0; ii < motor_vals; ii++) {
        EXPECT_EQ(msg[ii], ii);
    }
}

}  // namespace
}  // namespace comms
}  // namespace cassie
