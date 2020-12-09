#include "cassie/comms/cassie_lcm_parsing.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

#include <gtest/gtest.h>

namespace cassie {
namespace {
using drake::MatrixCompareType;
using Eigen::Vector3d;
using Eigen::Vector4d;

GTEST_TEST(CassieLcmParsing, CheckConversionArrays) {
  const int MotorToState_check[10] = {7, 9, 11, 13, 21, 8, 10, 12, 14, 22};
  const int JointToState_check[4] = {15, 17, 16, 18};
  const int SpringToState_check[4] = {-1, 19, -1, 20};
  const int MotorUrdfToSimulink_check[10] = {0, 2, 4, 6, 8, 1, 3, 5, 7, 9};

  EXPECT_EQ(kCassieMotorToState.size(), 10);
  EXPECT_EQ(kCassieJointToState.size(), 4);
  EXPECT_EQ(kCassieSpringToState.size(), 4);
  EXPECT_EQ(kCassieMotorUrdfToSimulink.size(), 10);

  for (int ii = 0; ii < 10; ii++) {
    EXPECT_EQ(kCassieMotorToState[ii], MotorToState_check[ii]);
    EXPECT_EQ(kCassieMotorUrdfToSimulink[ii], MotorUrdfToSimulink_check[ii]);
  }
  for (int ii = 0; ii < 4; ii++) {
    EXPECT_EQ(kCassieJointToState[ii], JointToState_check[ii]);
    EXPECT_EQ(kCassieSpringToState[ii], SpringToState_check[ii]);
  }
}

}  // namespace
}  // namespace cassie
