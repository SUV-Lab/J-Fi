#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

#include "jfi_comm.hpp"

using namespace std::chrono_literals;

/**
 * @class SerialCommNode
 * @brief ROS2 node example that uses the JFiComm library
 *
 *  - Subscribes to /to_serial -> sends data using the library
 *  - When a MAVLink message is received, converts it and publishes to /from_serial
 */
class SerialCommNode : public rclcpp::Node
{
public:
  enum TID{
    TID_STRING = 1,
    TID_INT = 2,
  };

public:
  SerialCommNode();
  ~SerialCommNode();

private:
  /**
   * @brief Called when a MAVLink message is fully parsed by the library
   */
  void handleMessage(const int tid, const std::vector<uint8_t> & msg);

private:
  JFiComm       jfi_comm_;

  // Parameters
  std::string   port_name_;
  int           baud_rate_;

  // ROS
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_to_serial_string;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_to_serial_int;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_from_serial_string_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_from_serial_int_;
  rclcpp::TimerBase::SharedPtr main_timer_;
};
