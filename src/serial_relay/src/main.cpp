#include <rclcpp/rclcpp.hpp>
#include "relay_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
