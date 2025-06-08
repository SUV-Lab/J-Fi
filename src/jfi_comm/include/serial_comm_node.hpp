#ifndef SERIAL_COMM_NODE_HPP
#define SERIAL_COMM_NODE_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_status.hpp>

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
    TID_TRAJECTORY_SETPOINT = 1,
    // TID_VEHICLE_STATUS = 2
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
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_to_serial_trajectory_setpoint_;
  
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_from_serial_trajectory_setpoint_;
};

#endif  // SERIAL_COMM_NODE_HPP
