#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
    TID_HELLO = 1,
    TID_TEST = 2,
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_to_serial_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_from_serial_;
  rclcpp::TimerBase::SharedPtr main_timer_;
};
