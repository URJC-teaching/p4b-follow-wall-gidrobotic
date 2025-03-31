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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "move/controller.hpp"
#include <cmath>

using namespace std::chrono_literals;

namespace move
{

Controller::Controller() : Node("controller") {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_filtered", 10, std::bind(&Controller::laser_callback, this, std::placeholders::_1));
}

void Controller::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto cmd = geometry_msgs::msg::Twist();

    int total_ranges = msg->ranges.size();
    int front_start = total_ranges / 360 * 20;
    int front_end = total_ranges - total_ranges / 360 * 20;
    int left_start = total_ranges / 4 - (total_ranges / 360 * 40);
    int left_end = total_ranges / 4 + (total_ranges / 360 * 40);
    int right_start = (total_ranges / 4 * 3) - (total_ranges / 360 * 40);
    int right_end = (total_ranges / 4 * 3) + (total_ranges / 360 * 40);

    float min_front = msg->ranges[front_start];
    float max_front = msg->ranges[front_start];
    for (int i = front_start; i > total_ranges / 360 * 0; --i) {
        if (std::isfinite(msg->ranges[i])) {
            if (msg->ranges[i] < min_front) {
                min_front = msg->ranges[i];
            } 
            else if (msg->ranges[i] > max_front) {
                max_front = msg->ranges[i];
            }
        }
    }
    for (int i = front_end; i < total_ranges; ++i) {
        if (std::isfinite(msg->ranges[i])) {
            if (std::isfinite(msg->ranges[i])) {
                if (msg->ranges[i] < min_front) {
                    min_front = msg->ranges[i];
                } 
                else if (msg->ranges[i] > max_front) {
                    max_front = msg->ranges[i];
                }
            }
        }
    }

    float min_left = msg->ranges[left_start];
    float max_left = msg->ranges[left_start];
    for (int i = left_start; i < left_end; ++i) {
         if (std::isfinite(msg->ranges[i])) {
            if (msg->ranges[i] < min_left) {
                min_left = msg->ranges[i];
            } 
            else if (msg->ranges[i] > max_left) {
                max_left = msg->ranges[i];
            }
        }
    }
    
    float min_right = msg->ranges[right_start];
    float max_right = msg->ranges[right_start];
    for (int i = right_start; i < right_end; ++i) {
         if (std::isfinite(msg->ranges[i])) {
            if (msg->ranges[i] < min_right) {
                min_right = msg->ranges[i];
            } 
            else if (msg->ranges[i] > max_right) {
                max_right = msg->ranges[i];
            }
        }
    
    
    
    
    
    RCLCPP_INFO(this->get_logger(), "Distancias Frente -> MIN: %.2f m | MAX: %.2f m", min_front, max_front);
    RCLCPP_INFO(this->get_logger(), "Distancias Izquierda -> MIN: %.2f m | MAX: %.2f m", min_left, max_left);
    RCLCPP_INFO(this->get_logger(), "Distancias Derecha -> MIN: %.2f m | MAX: %.2f m", min_right, max_right);

    // Estado actual
    switch (current_state_) {
        case State::BUSCAR_PARED:
            RCLCPP_INFO(this->get_logger(), "Estado actual: BUSCAR_PARED");
            if (min_front < umbral_obstaculo_ && max_front > umbral_pared_max_) {
                current_state_ = State::EVITAR_OBSTACULO;
            } 
            else if (min_left < distancia_pared_deseada_ && max_left < distancia_pared_deseada_) {
                current_state_ = State::SEGUIR_PARED;
            } 
            else if (min_front < umbral_obstaculo_ && max_front < umbral_pared_max_) {
                current_state_ = State::AJUSTAR_DISTANCIA;
            } 
            else {
                cmd.linear.x = 0.2;
                cmd.angular.z = 0.0;
            }
            break;

        case State::SEGUIR_PARED:
            RCLCPP_INFO(this->get_logger(), "Estado actual: SEGUIR_PARED");
            if (min_front < 1.0) {
                current_state_ = State::EVITAR_OBSTACULO;
            } 
            else if (min_left < 0.5 || min_left > 1.5) {
                current_state_ = State::AJUSTAR_DISTANCIA;
            } 
            else {
                cmd.linear.x = 0.2;
                cmd.angular.z = 0.0;
            }
            break;

        case State::AJUSTAR_DISTANCIA:
            RCLCPP_INFO(this->get_logger(), "Estado actual: AJUSTAR_DISTANCIA");
            if (min_left > 1.1) {
                cmd.linear.x = 0.1;
                cmd.angular.z = 0.3;  // Girar a la izquierda
            } 
            else if (min_left < 0.9) {
                cmd.linear.x = 0.1;
                cmd.angular.z = -0.3; // Girar a la derecha
            }

            if (min_left > 0.9 || min_left < 1.1) {
                current_state_ = State::SEGUIR_PARED;
            } 
            break;

        case State::EVITAR_OBSTACULO:
            RCLCPP_INFO(this->get_logger(), "Estado actual: EVITAR_OBSTACULO");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.2; 
            if (min_front > umbral_pared_max_ && max_front > umbral_pared_max_) {
                current_state_ = State::BUSCAR_PARED;
            }
            break;
            
        default:
            RCLCPP_INFO(this->get_logger(), "Estado actual: default");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            break;
            
    }

    RCLCPP_INFO(this->get_logger(), "Velocidades -> Linear: %.2f m/s | Angular: %.2f rad/s ", cmd.linear.x, cmd.angular.z);

    publisher_->publish(cmd);
}

}
}  // namespace move

