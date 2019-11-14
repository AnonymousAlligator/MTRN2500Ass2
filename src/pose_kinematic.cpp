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
    , zid_{zid}
{
    auto callback = std::bind(
        &PoseKinematic::velocity_callback, this, std::placeholders::_1);
    this->velocity_input_ =
        create_subscription<geometry_msgs::msg::TwistStamped>(
            "/" + zid + "/velocity", 10, callback);
    this->pose_output_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        std::string{"/" + zid + "/pose"}, 10);
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
