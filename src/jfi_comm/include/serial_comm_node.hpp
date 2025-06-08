#ifndef SERIAL_COMM_NODE_HPP
#define SERIAL_COMM_NODE_HPP

#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <uwb_msgs/msg/ranging.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "jfi_comm.hpp"

using namespace std::chrono_literals;

class SerialCommNode : public rclcpp::Node
{
public:
  enum TID{
    TID_RANGING = 1,
    TID_TRAJECTORY = 2,
  };

  SerialCommNode();
  ~SerialCommNode();

private:
  /**
   * @brief Handle incoming JFI data.
   * @param tid Message type ID (Ranging or TrajectorySetpoint)
   * @param src_sysid MAVLink sysid of the sending drone
   * @param data Serialized ROS2 message bytes
   */
  void handleMessage(int tid, uint8_t src_sysid, const std::vector<uint8_t>& data);

  JFiComm jfi_comm_;

  // Serial port settings
  std::string port_name_;
  int baud_rate_;

  // MAVLink IDs for this node
  uint8_t system_id_;
  uint8_t component_id_;

  // Prefix for JFI topics (e.g., "drone1/jfi/")
  std::string topic_prefix_jfi_;

  // List of all drone system IDs in the network
  std::vector<int> system_id_list_;

  // Subscriptions for outgoing messages to serial
  rclcpp::Subscription<uwb_msgs::msg::Ranging>::SharedPtr            sub_to_serial_ranging_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_to_serial_trajectory_;

  // Publishers for incoming messages from serial, keyed by source sysid
  std::map<int, rclcpp::Publisher<uwb_msgs::msg::Ranging>::SharedPtr>            pub_ranging_map_;
  std::map<int, rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr> pub_trajectory_map_;
};

#endif  // SERIAL_COMM_NODE_HPP
