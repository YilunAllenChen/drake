#include <array>

namespace cassie {

const std::array<int, 10> kCassieMotorToState = {7, 9,  11, 13, 21,
                                                 8, 10, 12, 14, 22};
const std::array<int, 4> kCassieJointToState = {15, 17, 16, 18};
const std::array<int, 4> kCassieSpringToState = {-1, 19, -1, 20};
const std::array<int, 10> kCassieMotorUrdfToSimulink = {0, 2, 4, 6, 8,
                                                        1, 3, 5, 7, 9};

}  // namespace cassie
