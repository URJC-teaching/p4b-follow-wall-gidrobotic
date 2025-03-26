#include "rclcpp/rclcpp.hpp"
#include "move/controller.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<move::Controller>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
