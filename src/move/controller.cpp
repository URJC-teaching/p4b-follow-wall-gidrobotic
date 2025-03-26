// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "move/controller.hpp"
#include <cmath>

namespace move {

Controller::Controller() : rclcpp::Node("controller")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_raw", 10, std::bind(&Controller::laser_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Controller node started.");
}

void Controller::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    bool obstacle_detected = false;
    int total_ranges = msg->ranges.size();
    int center_start = total_ranges / 2 - 15;
    int center_end = total_ranges / 2 + 15;

    for (int i = center_start; i < center_end; ++i)
    {
        float range = msg->ranges[i];
        if (std::isfinite(range) && range > 3)
        {
            obstacle_detected = true;
            break;
        }
    }

    geometry_msgs::msg::Twist cmd;

    if (obstacle_detected)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "🔴 Obstacle detected! Stopping.");
    }
    else
    {
        cmd.linear.x = 0.2;
        cmd.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "🟢 Path clear. Moving forward.");
    }

    publisher_->publish(cmd);
}

} // namespace move
