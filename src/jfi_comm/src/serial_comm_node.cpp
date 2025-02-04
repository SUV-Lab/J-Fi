#include "serial_comm_node.hpp"
#include <rclcpp/serialization.hpp>

#include <functional>

using namespace std::chrono_literals;

SerialCommNode::SerialCommNode()
: Node("serial_comm_node")
{
  // declare and get parameters
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  port_name_ = this->get_parameter("port_name").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();

  // init JFiComm
  jfi_comm_.init(
    std::bind(&SerialCommNode::handleMessage, this, std::placeholders::_1, std::placeholders::_2),
    port_name_, baud_rate_
  );

  // create subscriptions
  sub_to_serial_ = this->create_subscription<std_msgs::msg::String>(
    "hello_topic",
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      auto serialized_data = jfi_comm_.serialize_message(msg);
      jfi_comm_.send(TID_HELLO, serialized_data);
    }
  );

  // create publishers
  pub_from_serial_ = this->create_publisher<std_msgs::msg::String>("from_serial", 10);
}

SerialCommNode::~SerialCommNode()
{
  jfi_comm_.closePort();
}

void SerialCommNode::handleMessage(const int tid, const std::vector<uint8_t> & data)
{
  switch (tid) {
    case TID_HELLO:
    {
      std_msgs::msg::String hello_msg = jfi_comm_.deserialize_message<std_msgs::msg::String>(data);
      pub_from_serial_->publish(hello_msg);
    }
    break;
    case TID_TEST:
    {

    }
    break;
    default:
    break;
  }
}

