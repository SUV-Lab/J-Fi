#ifndef SERIAL_COMM_NODE_HPP
#define SERIAL_COMM_NODE_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <path_manager/msg/poly_traj.hpp>
#include <path_manager/msg/formation_command.hpp>

#include "jfi_comm.hpp"

using namespace std::chrono_literals;

/**
 * @class SerialCommNode
 * @brief ROS2 node example that uses the JFiComm library to bridge ROS2 topics and MAVLink serial communication.
 *
 *  - Subscribes to /to_serial -> sends data using the library
 *  - When a MAVLink message is received, converts it and publishes to /from_serial
 */
class SerialCommNode : public rclcpp::Node
{
public:
  enum TID{
    TID_POLY_TRAJ = 1,
    TID_FORMATION_COMMAND = 2
  };

public:
  SerialCommNode();
  ~SerialCommNode();

private:
  /**
   * @brief Callback invoked when a MAVLink message is received.
   *
   * @param tid Message type identifier.
   * @param msg Received message data as a byte vector.
   */
  void handleMessage(const int tid, const std::vector<uint8_t>& msg);

private:
  JFiComm       jfi_comm_;

  // Parameters
  std::string   port_name_;
  int           baud_rate_;
  uint8_t       system_id_;
  uint8_t       component_id_;

  // ROS subscriptions and publishers
  rclcpp::Subscription<path_manager::msg::PolyTraj>::SharedPtr sub_to_serial_poly_traj_;
  rclcpp::Publisher<path_manager::msg::PolyTraj>::SharedPtr pub_from_serial_poly_traj_;

  rclcpp::Subscription<path_manager::msg::FormationCommand>::SharedPtr sub_to_serial_formation_cmd_;
  rclcpp::Publisher<path_manager::msg::FormationCommand>::SharedPtr pub_from_serial_formation_cmd_;

  // Service for dynamic subscription control
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_formation_cmd_send_service_;
  bool formation_cmd_send_enabled_;

  // Service callback
  void enableFormationCommandSendCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

#endif  // SERIAL_COMM_NODE_HPP
