#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "jfi_comm.hpp"

using namespace std::chrono_literals;

/**
 * @class SerialCommNode
 * @brief ROS2 node example that uses the JFiComm library
 * 
 *  - Subscribes to /to_serial (String) -> sends data using the library
 *  - Periodically calls checkSendBuffer() and readMavlinkMessages() in a timer
 *  - When a MAVLink message is received, converts it to a ROS String and publishes to /from_serial
 */
class SerialCommNode : public rclcpp::Node
{
public:
  SerialCommNode()
  : Node("serial_comm_node")
  {
    // ---------------------------
    // 1) Declare and get parameters
    // ---------------------------
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);

    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    // ---------------------------
    // 2) Open the serial port
    // ---------------------------
    bool ok = jfi_comm_.openPort(port_name_, baud_rate_);
    if(!ok) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open port: %s (baud=%d)",
                   port_name_.c_str(), baud_rate_);
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Opened port: %s (baud=%d)",
                port_name_.c_str(), baud_rate_);

    // ---------------------------
    // 3) Register the receive callback
    // ---------------------------
    jfi_comm_.setReceiveCallback(
      [this](const mavlink_message_t & msg)
      {
        this->handleMavlinkMessage(msg);
      }
    );

    // ---------------------------
    // 4) Create ROS2 Pub/Sub
    // ---------------------------
    // /to_serial Subscribe
    sub_to_serial_ = this->create_subscription<std_msgs::msg::String>(
      "to_serial", 
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        jfi_comm_.send(msg->data);
        RCLCPP_INFO(this->get_logger(), 
                    "[SUB] to_serial: '%s' -> send()", 
                    msg->data.c_str());
      }
    );

    // /from_serial Publish
    pub_from_serial_ = this->create_publisher<std_msgs::msg::String>(
      "from_serial", 
      10
    );

    // ---------------------------
    // 5) Create a timer for sending buffer and reading data
    // ---------------------------
    main_timer_ = this->create_wall_timer(
      50ms, 
      [this]() {
        jfi_comm_.checkSendBuffer();
        jfi_comm_.readMavlinkMessages();
      }
    );

    RCLCPP_INFO(this->get_logger(), 
                "SerialCommNode started with port=%s, baud=%d",
                port_name_.c_str(), baud_rate_);
  }

  ~SerialCommNode()
  {
    jfi_comm_.closePort();
  }

private:
  /**
   * @brief Called when a MAVLink message is fully parsed by the library
   */
  void handleMavlinkMessage(const mavlink_message_t & msg)
  {
    if(msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
      mavlink_statustext_t st;
      mavlink_msg_statustext_decode(&msg, &st);

      std::string text((char*)st.text);

      // /from_serial Publish
      auto ros_msg = std_msgs::msg::String();
      ros_msg.data = text;
      pub_from_serial_->publish(ros_msg);

      RCLCPP_INFO(this->get_logger(),
                  "[RECV] MAVLINK_MSG_ID_STATUSTEXT: '%s'",
                  text.c_str());
    }
    else {
      RCLCPP_INFO(this->get_logger(),
                  "[RECV] msgid=%u, len=%u",
                  msg.msgid, msg.len);
    }
  }

private:
  JFiComm jfi_comm_;

  // Parameters
  std::string port_name_;
  int baud_rate_;

  // ROS
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_to_serial_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_from_serial_;
  rclcpp::TimerBase::SharedPtr main_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialCommNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
