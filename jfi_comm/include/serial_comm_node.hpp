#ifndef SERIAL_COMM_NODE_HPP
#define SERIAL_COMM_NODE_HPP

#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "jfi_comm/msg/swarm_comm.hpp"
#include "jfi_comm.hpp"


/**
 * @class SerialCommNode
 * @brief An example node demonstrating how to use the JFiComm library.
 */
class SerialCommNode : public rclcpp::Node
{
public:
  SerialCommNode();
  ~SerialCommNode();

private:
  /**
   * @brief Callback function that processes all incoming messages from JFiComm.
   */
  void handleMessage(uint8_t seq, uint8_t tid,
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
  rclcpp::Publisher<jfi_comm::msg::SwarmComm>::SharedPtr pub_packet_;
  rclcpp::Subscription<jfi_comm::msg::SwarmComm>::SharedPtr sub_packet_;
};

#endif  // SERIAL_COMM_NODE_HPP