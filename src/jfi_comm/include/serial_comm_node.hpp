#ifndef SERIAL_COMM_NODE_HPP
#define SERIAL_COMM_NODE_HPP

#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "jfi_comm.hpp"

using namespace std::chrono_literals;

/**
 * @class SerialCommNode
 * @brief An example node demonstrating how to use the JFiComm library.
 */
class SerialCommNode : public rclcpp::Node
{
public:
  /**
   * @brief Defines the unique identifiers for different message types being sent over MAVLink.
   */
  enum TID : uint8_t
  {
    TID_ROS_STRING   = 1,
  };

  SerialCommNode();
  ~SerialCommNode();

private:
  /**
   * @brief Callback function that processes all incoming messages from JFiComm.
   */
  void handleMessage(uint8_t tid,
                     uint8_t src_sysid,
                     const std::vector<uint8_t>& data);

  /* ---------- Members ---------------------------------------------------- */
  JFiComm jfi_comm_;

  // Serial port parameters
  std::string port_name_;
  int         baud_rate_;

  // Local MAVLink identity
  uint8_t system_id_;
  uint8_t component_id_;

  /* ROS interfaces -------------------------------------------------------- */
  // Publisher for incoming data from another device
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string_;

  // Subscriber for data to be sent to another device
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_string_;
};

#endif  // SERIAL_COMM_NODE_HPP