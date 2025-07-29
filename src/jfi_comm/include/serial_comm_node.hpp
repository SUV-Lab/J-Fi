#ifndef SERIAL_COMM_NODE_HPP
#define SERIAL_COMM_NODE_HPP

#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "jfi_comm.hpp"

using namespace std::chrono_literals;

/**
 * @class SerialCommNode
 * @brief An example node demonstrating how to use the JFiComm library.
 * * - Periodically sends a std_msgs::msg::String message via serial.
 * - Listens for incoming serial data, deserializes it into ROS messages,
 * and publishes them to ROS topics.
 */
class SerialCommNode : public rclcpp::Node
{
public:
  /**
   * @brief Defines the unique identifiers for different message types
   * being sent over MAVLink.
   */
  enum TID : uint8_t
  {
    TID_ROS_STRING   = 1,
    TID_ROS_FLOATS   = 2,
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

  /**
   * @brief Timer callback to periodically send a string message.
   */
  void sendStringTimerCallback();

  /* ---------- Members ---------------------------------------------------- */
  JFiComm jfi_comm_;

  // Serial port parameters
  std::string port_name_;
  int         baud_rate_;

  // Local MAVLink identity
  uint8_t system_id_;
  uint8_t component_id_;

  /* ROS interfaces -------------------------------------------------------- */
  // Publisher for incoming Float64MultiArray data from another device
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_float_array_;
  
  // Publisher for incoming String data from another device
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string_;

  // Timer to trigger sending our own string message
  rclcpp::TimerBase::SharedPtr send_string_timer_;

  // Counter for the message to be sent
  size_t send_count_ = 0;
};

#endif  // SERIAL_COMM_NODE_HPP
