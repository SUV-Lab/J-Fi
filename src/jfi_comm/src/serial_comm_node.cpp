#include "serial_comm_node.hpp"
#include <functional>

SerialCommNode::SerialCommNode()
: Node("serial_comm_node")
{
  // Declare and get parameters.
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  port_name_ = this->get_parameter("port_name").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();

  RCLCPP_INFO(this->get_logger(), "Starting SerialCommNode with port: %s, baud_rate: %d",
              port_name_.c_str(), baud_rate_);

  // Initialize JFiComm.
  if (!jfi_comm_.init(
        std::bind(&SerialCommNode::handleMessage, this, std::placeholders::_1, std::placeholders::_2),
        port_name_, baud_rate_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize JFiComm on port: %s", port_name_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "JFiComm initialized successfully.");
  }

  // Create subscription for outgoing messages to serial.
  sub_to_serial_ = this->create_subscription<std_msgs::msg::String>(
    "to_serial", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      auto serialized_data = jfi_comm_.serialize_message(msg);
      if (!serialized_data.empty()) {
        jfi_comm_.send(TID_HELLO, serialized_data);
        RCLCPP_INFO(this->get_logger(), "Sent message via serial: %s", msg->data.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to serialize message for serial transmission.");
      }
    }
  );

  // Create publisher for incoming messages from serial.
  pub_from_serial_ = this->create_publisher<std_msgs::msg::String>("from_serial", 10);
}

SerialCommNode::~SerialCommNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down SerialCommNode, closing serial port.");
  jfi_comm_.closePort();
}

void SerialCommNode::handleMessage(const int tid, const std::vector<uint8_t> & data)
{
  switch (tid) {
    case TID_HELLO:
    {
      try {
        std_msgs::msg::String hello_msg = jfi_comm_.deserialize_message<std_msgs::msg::String>(data);
        pub_from_serial_->publish(hello_msg);
        RCLCPP_INFO(this->get_logger(), "Received and published TID_HELLO message: %s", hello_msg.data.c_str());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to deserialize TID_HELLO message: %s", e.what());
      }
    }
    break;
    case TID_TEST:
    {
      RCLCPP_WARN(this->get_logger(), "Received TID_TEST message, but no handler is implemented.");
    }
    break;
    default:
      RCLCPP_WARN(this->get_logger(), "Received unknown message TID: %d", tid);
      break;
  }
}
