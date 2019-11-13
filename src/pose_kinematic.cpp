// Copyright 2019 Zhihao Zhang License MIT

#include "pose_kinematic.hpp"

#include "student_helper.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <utility>

namespace assignment2
{
PoseKinematic::PoseKinematic(
    std::string const & zid, std::chrono::milliseconds const refresh_period)
    : rclcpp::Node(helper::pose_node_name(zid))

{
    // TODO(STUDENT): CODE HERE
}

auto PoseKinematic::velocity_callback(
    geometry_msgs::msg::TwistStamped::UniquePtr input_message) -> void
{
    // TODO(STUDENT): CODE HERE
}

auto PoseKinematic::pose_callback() -> void
{
    // TODO(STUDENT): CODE HERE
}
} // namespace assignment2
