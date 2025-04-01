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
#include <limits>

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
    }
    RCLCPP_INFO(this->get_logger(), "Frente: %.2f - %.2f | Izquierda: %.2f - %.2f | Derecha: %.2f - %.2f",
                min_front, max_front, min_left, max_left, min_right, max_right);

    switch (current_state_) {
        case State::BUSCAR_PARED:
            RCLCPP_INFO(this->get_logger(), "Estado: BUSCAR_PARED");
            if (( min_front < umbral_pared_min_ ) && ( max_front > umbral_pared_max_ )) {
                current_state_ = State::EVITAR_OBSTACULO;
            } else if (( min_left < (distancia_pared_deseada_ + 0.1 )) && ( max_left > (distancia_pared_deseada_ + 0.1 ))) {
                current_state_ = State::SEGUIR_PARED;
            } else if (( min_front > umbral_pared_min_ ) && ( max_front < umbral_pared_max_ )) {
                current_state_ = State::AJUSTAR_DISTANCIA;
            } else {
                cmd.linear.x = 0.2;
                cmd.angular.z = 0.0;
            }
            break;

        case State::SEGUIR_PARED:
            RCLCPP_INFO(this->get_logger(), "Estado: SEGUIR_PARED");
            if (min_front < umbral_pared_min_) {
                current_state_ = State::EVITAR_OBSTACULO;
            } else if ((min_left < (distancia_pared_deseada_ - 0.5)) || (min_left > (distancia_pared_deseada_ + 0.5))) {
                current_state_ = State::AJUSTAR_DISTANCIA;
            } else {
                cmd.linear.x = 0.2;
                cmd.angular.z = 0.0;
            }
            break;

        case State::AJUSTAR_DISTANCIA:
            RCLCPP_INFO(this->get_logger(), "Estado: AJUSTAR_DISTANCIA");
            if ((min_front > (umbral_pared_min_ - 0.1)) && (max_front < (umbral_pared_max_ + 0.1))) {
                cmd.linear.x = 0.0;
                cmd.angular.z = -0.3;
                }
            else if (min_front < umbral_pared_min_) {
                current_state_ = State::EVITAR_OBSTACULO;
            } 
            else if (min_left < (distancia_pared_deseada_ - 0.1)) {
                cmd.linear.x = 0.1;
                cmd.angular.z = -0.3;
            } 
            else if (min_left > (distancia_pared_deseada_ + 0.1)) {
                cmd.linear.x = 0.1;
                cmd.angular.z = 0.3;
            }

            if ((min_left > (distancia_pared_deseada_ - 0.1)) && (min_left < (distancia_pared_deseada_ + 0.1))) {
                current_state_ = State::SEGUIR_PARED;
            }
            break;

        case State::EVITAR_OBSTACULO:
            RCLCPP_INFO(this->get_logger(), "Estado: EVITAR_OBSTACULO");
            cmd.linear.x = 0.0;
            cmd.angular.z = -0.2;
            if ((min_front > umbral_pared_max_) && (max_front > umbral_pared_max_)) {
                current_state_ = State::BUSCAR_PARED;
            }
            break;

        default:
            RCLCPP_INFO(this->get_logger(), "Estado: DEFAULT");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            break;
    }

    RCLCPP_INFO(this->get_logger(), "Velocidades -> Linear: %.2f m/s | Angular: %.2f rad/s",
                cmd.linear.x, cmd.angular.z);

    publisher_->publish(cmd);
}

} // namespace move

