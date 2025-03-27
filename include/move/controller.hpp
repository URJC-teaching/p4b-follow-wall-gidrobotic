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


#ifndef MOVE_CONTROLLER_HPP_
#define MOVE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // Incluir el mensaje del LIDAR

namespace move
{

enum class State
{
    BUSCAR_PARED,    // Buscar una pared
    SEGUIR_PARED,    // Seguir la pared izquierda
    AJUSTAR_DISTANCIA, // Ajustar la distancia a la pared
    EVITAR_OBSTACULO  // Evitar un obstáculo delante
};

class Controller : public rclcpp::Node
{
public:
    Controller();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    State current_state_ = State::BUSCAR_PARED;

    float distancia_pared_deseada_ = 1.0;  // Distancia deseada a la pared (en metros)
    float umbral_obstaculo_ = 0.5;        // Umbral para detectar un obstáculo
    float umbral_pared_max_ = 2.0;        // Si la distancia a la pared es mayor a este valor, buscar pared

    void update_movement(float left, float front, geometry_msgs::msg::Twist &cmd);
};

}  // namespace move

#endif
 // CONTROLLER_HPP_

