// Copyright 2019 Zhihao Zhang License MIT

#include "joystick_listener.hpp"

#include "student_helper.hpp"

#include <memory>
#include <string>
#include <utility>

namespace assignment2
{
JoystickListener::JoystickListener(std::string const & zid, JoystickConfig config)
    // setting the values to be used for the publisher/subscriber and config values
    : rclcpp::Node{helper::joy_node_name(zid)}
    , zid_{zid}
    , config_{config}
{
    // setting up publisher and subscriber
    auto callback = std::bind(&JoystickListener::joy_message_callback,this,std::placeholders::_1);
    joystick_input_ = this->create_subscription<sensor_msgs::msg::Joy>("/"+zid+"/joy",10,callback);
    this->acceleration_output_= create_publisher<geometry_msgs::msg::AccelStamped>
        (std::string{"/z0000000/acceleration"}, 10);
}

// callback
auto JoystickListener::joy_message_callback(sensor_msgs::msg::Joy::UniquePtr joy_message) -> void
{
    // putting axes values into a vector
    std::vector<float> axesValues{joy_message->axes.at(config_.speed_plus_axis), joy_message->axes.at(config_.speed_minus_axis),
        joy_message->axes.at(config_.steering_axis)};

    // lambda function that prints values
    auto print = [](const int& i) {std::cout <<'\t'<<i;};
 
    // printing axes values
    std::cout << "axes status:";
    std::for_each(axesValues.begin(), axesValues.end(), print);
    std::cout << '\n';  

    // putting button presses into a vector
    std::vector<double> buttonValues{joy_message->buttons.at(0), joy_message->buttons.at(1),
        joy_message->buttons.at(2),joy_message->buttons.at(3),joy_message->buttons.at(4),
            joy_message->buttons.at(5),joy_message->buttons.at(6),joy_message->buttons.at(7),
                joy_message->buttons.at(8),joy_message->buttons.at(9),joy_message->buttons.at(10)};
    
    // if button pressed it will be 1
    int target = 1;
    // counting and printing number of buttons pressed
    int buttonsPressed = std::count(buttonValues.begin(), buttonValues.end(), target);
    std::cout << "Total number of buttons pressed is: " << buttonsPressed << "\n";
    // accounting for deadzone in joystick in plus-minus direction
    float accelX = 0;
    if ((-config_.speed_deadzone <= axesValues.at(0)) && (axesValues.at(0)<= config_.speed_deadzone)){
        accelX = 0;
    } else if (axesValues.at(0) > config_.speed_deadzone){
        accelX = (axesValues.at(0)-config_.speed_deadzone)*(1/(1-config_.speed_deadzone));
    } else if (axesValues.at(1) < -config_.speed_deadzone){
        accelX = (axesValues.at(1)+config_.speed_deadzone)*(1/(1-config_.speed_deadzone));
    }
    //accounting for deadzone in joystick in steering direction
    float accelAng = 0;
    if ((-config_.steering_deadzone <= axesValues.at(2)) && (axesValues.at(2)<= config_.steering_deadzone)){
        accelAng = 0;
    } else if (axesValues.at(2) > config_.steering_deadzone){
        accelAng = (axesValues.at(2)-config_.speed_deadzone);
    } else if (axesValues.at(2) < -config_.speed_deadzone){
        accelAng = (axesValues.at(2)+config_.speed_deadzone);
    }
    // mapping to positive linear acceleration
    float accelPosX = 0.5*accelX + 0.5;
    // mapping to negative linear acceleration
    float accelNegX = 0.5*accelX - 0.5;

    // net linear acceleration
    float netAccel = accelPosX + accelNegX;

        auto acc_msg = std::make_unique<geometry_msgs::msg::AccelStamped>();
        acc_msg->header.frame_id = "zid";
        //acc_msg->header.stamp = "Time";
        acc_msg->accel.linear.x = accelX;
        acc_msg->accel.angular.z = accelAng;
        acceleration_output_->publish(std::move(acc_msg));
}
} // namespace assignment2
