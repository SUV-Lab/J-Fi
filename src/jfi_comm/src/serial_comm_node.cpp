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
  sub_to_serial_string = this->create_subscription<std_msgs::msg::String>(
    "to_serial_string", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      auto serialized_data = jfi_comm_.serialize_message(msg);
      if (!serialized_data.empty()) {
        jfi_comm_.send(TID_STRING, serialized_data);
        RCLCPP_INFO(this->get_logger(), "Sent message via serial: %s", msg->data.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to serialize message for serial transmission.");
      }
    }
  );
  sub_to_serial_int = this->create_subscription<std_msgs::msg::Int32>(
    "to_serial_int", 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      auto serialized_data = jfi_comm_.serialize_message(msg);
      if (!serialized_data.empty()) {
        jfi_comm_.send(TID_INT, serialized_data);
        RCLCPP_INFO(this->get_logger(), "Sent message via serial: %d", msg->data);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to serialize message for serial transmission.");
      }
    }
  );

  // Create publisher for incoming messages from serial.
  pub_from_serial_string_ = this->create_publisher<std_msgs::msg::String>("from_serial_string", 10);
  pub_from_serial_int_ = this->create_publisher<std_msgs::msg::Int32>("from_serial_int", 10);
}

SerialCommNode::~SerialCommNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down SerialCommNode, closing serial port.");
  jfi_comm_.closePort();
}

void SerialCommNode::handleMessage(const int tid, const std::vector<uint8_t> & data)
{
  switch (tid) {
    case TID_STRING:
    {
      try {
        std_msgs::msg::String string_msg = jfi_comm_.deserialize_message<std_msgs::msg::String>(data);
        pub_from_serial_string_->publish(string_msg);
        RCLCPP_INFO(this->get_logger(), "Received and published TID_STRING message: %s", string_msg.data.c_str());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to deserialize TID_STRING message: %s", e.what());
      }
    }
    break;
    case TID_INT:
    {
      try {
        // Deserialize the incoming data as an Int32 message.
        std_msgs::msg::Int32 int_msg = jfi_comm_.deserialize_message<std_msgs::msg::Int32>(data);
        pub_from_serial_int_->publish(int_msg);
        RCLCPP_INFO(this->get_logger(), "Received and published TID_INT message: %d", int_msg.data);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to deserialize TID_INT message: %s", e.what());
      }
    }
    break;
    default:
    {
      RCLCPP_WARN(this->get_logger(), "Received unknown message TID: %d", tid);
    }
    break;
  }
}
