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

using namespace std::chrono_literals;

namespace move
{

Controller::Controller() : Node("controller") {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_raw", 10, std::bind(&Controller::laser_callback, this, std::placeholders::_1));
}

void Controller::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto cmd = geometry_msgs::msg::Twist();

    // Índices para 30 grados a la izquierda y al frente
    int front_start = msg->ranges.size() / 2 - (msg->ranges.size() / 360 * 15); // 15 grados a la izquierda del centro
    int front_end = msg->ranges.size() / 2 + (msg->ranges.size() / 360 * 15);   // 15 grados a la derecha del centro
    int left_start = msg->ranges.size() / 4 - (msg->ranges.size() / 360 * 15);   // 15 grados hacia la izquierda
    int left_end = msg->ranges.size() / 4 + (msg->ranges.size() / 360 * 15);     // 15 grados hacia la izquierda

    // Promediar las distancias de los 30 grados a la izquierda y al frente
    float front = 0.0;
    for (int i = front_start; i < front_end; ++i) {
        front += msg->ranges[i];
    }
    front /= (front_end - front_start);  // Promedio de distancias al frente

    float left = 0.0;
    for (int i = left_start; i < left_end; ++i) {
        left += msg->ranges[i];
    }
    left /= (left_end - left_start);  // Promedio de distancias a la izquierda

    update_movement(left, front, cmd);

    publisher_->publish(cmd);//Envía el mensaje de velocidad a /cmd_vel.
  RCLCPP_INFO(this->get_logger(), "Publicando velocidad: x=%.2f, z=%.2f", cmd.linear.x, cmd.angular.z);
}

void Controller::update_movement(float left, float front, geometry_msgs::msg::Twist &cmd)
{
    switch (current_state_)
    {
    case State::BUSCAR_PARED:
        if (left < distancia_pared_deseada_ + 0.2) {
            current_state_ = State::SEGUIR_PARED;
        }
        else if (front < umbral_obstaculo_) {
            current_state_ = State::EVITAR_OBSTACULO;
        }
        else{
            cmd.linear.x = 0.2;  // Avanzar
            cmd.angular.z = 0.0; // Sin giro
        }
        break;

    case State::SEGUIR_PARED:
        if (front < umbral_obstaculo_) {
            current_state_ = State::EVITAR_OBSTACULO;
        }
        else if (left > umbral_pared_max_)  {
            current_state_ = State::BUSCAR_PARED;
        }
        else if (std::abs(left - distancia_pared_deseada_) > 0.2) {
            current_state_ = State::AJUSTAR_DISTANCIA;
        }
        else
        {
            cmd.linear.x = 0.2;  // Seguir adelante
            cmd.angular.z = 0.0; // Sin giro
        }
        break;

    case State::AJUSTAR_DISTANCIA:
        if (left < distancia_pared_deseada_ - 0.1) {
            cmd.angular.z = -0.2;  // Alejarse un poco
        } 
        else if (left > distancia_pared_deseada_ + 0.1) {
            cmd.angular.z = 0.2;   // Acercarse un poco
        }
        else {
            current_state_ = State::SEGUIR_PARED;  // Volver a seguir la pared
        }
        cmd.linear.x = 0.1;  // Avanzar lentamente
        break;

    case State::EVITAR_OBSTACULO:
        if (front > umbral_obstaculo_ + 0.3) {
            current_state_ = State::BUSCAR_PARED;
        }
        else {
            cmd.linear.x = -0.2;   // Retrocede
            cmd.angular.z = 0.5;  // Girar a la izquierda
        }
        break;
        default:
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
      break;
    }
  
}


} // namespace move
