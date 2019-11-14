// Copyright 2019 Zhihao Zhang License MIT

#include "velocity_kinematic.hpp"

#include "student_helper.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <utility>

namespace assignment2
{
VelocityKinematic::VelocityKinematic(std::string const & zid,
    std::chrono::milliseconds const refresh_period, KinematicLimits config)
    : rclcpp::Node(helper::velocity_node_name(zid))
    , zid_{zid}
    , config_{config}
{
    // setting up publisher and subscriber
    auto callback = std::bind(
        &VelocityKinematic::acceleration_callback, this, std::placeholders::_1);
    this->acceleration_input_ =
        create_subscription<geometry_msgs::msg::AccelStamped>(
            "/" + zid + "/acceleration", 10, callback);
    this->velocity_output_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        std::string{"/z0000000/velocity"}, 10);
}

auto VelocityKinematic::acceleration_callback(
    geometry_msgs::msg::AccelStamped::UniquePtr input_message) -> void
{
    // TODO(STUDENT): CODE HERE
}

auto VelocityKinematic::velocity_callback() -> void
{
    // TODO(STUDENT): CODE HERE
}
} // namespace assignment2
