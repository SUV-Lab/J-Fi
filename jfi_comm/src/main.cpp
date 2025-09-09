#include <rclcpp/rclcpp.hpp>
#include "serial_comm_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialCommNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
