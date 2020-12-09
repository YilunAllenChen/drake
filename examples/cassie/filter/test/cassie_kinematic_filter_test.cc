#include "cassie/filter/cassie_kinematic_filter.h"

#include <gtest/gtest.h>

namespace cassie {
namespace filter {
namespace {

std::unique_ptr<CassieKinematicFilter> createFilter() {
    JointFilterParams joint_filter_params;
    ContactTriggerParams contact_trigger_params;
    KinematicFilterParams kinematic_filter_params;
    auto tree = getCassieTree<double>();
    auto cache = tree->CreateKinematicsCache();

    return std::make_unique<CassieKinematicFilter>(
        std::move(tree), cache, joint_filter_params, contact_trigger_params,
        kinematic_filter_params);
}

GTEST_TEST(CassieKinematicFilter, AccessData) {
    std::unique_ptr<CassieKinematicFilter> filter = createFilter();

    EXPECT_EQ(filter->get_full_state().rows(), kCassieStates);
    EXPECT_EQ(filter->get_contact().size(), 2);
}

GTEST_TEST(CassieKinematicFilter, Reset) {
    // TODO: Add this test
}

GTEST_TEST(CassieKinematicFilter, Update) {
    // TODO: Add this test
}

}  // namespace
}  // namespace filter
}  // namespace cassie
