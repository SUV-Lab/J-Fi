#include "relay_node.hpp"
#include "serial_comm.hpp"
#include <functional>

RelayNode::RelayNode()
: Node("realy_node")
{
    // Declare and get parameters.
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<std::string>("sub_topic_name", "relay_sub");
    this->declare_parameter<std::string>("pub_topic_name", "relay_pub");

    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();
    pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();

    RCLCPP_INFO(this->get_logger(),
                            "Starting RelayNode with port: %s, baud_rate: %d",
                            serial_port_.c_str(), baud_rate_);

    // Initialize serial comm.
    if (!serial_comm_.init(
                std::bind(&RelayNode::handleMessage, this, std::placeholders::_1),
                serial_port_, baud_rate_)) {
        RCLCPP_ERROR(this->get_logger(), "[SerialCommNode] Failed to initialize serial comm on port: %s", serial_port_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Serial comm initialized successfully.");
    }

    // Create subscriptions for outgoing messages.
    serial_raw_data_sub_ = this->create_subscription<serial_relay::msg::SerialRawData>(
        sub_topic_name_, 10,
        [this](const serial_relay::msg::SerialRawData::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "write serial data");
            serial_comm_.writeData(msg->data);
        }
    );

    // Create publishers for incoming messages.
    serial_raw_data_pub_ = this->create_publisher<serial_relay::msg::SerialRawData>(pub_topic_name_, 10);
}

RelayNode::~RelayNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down RelayNode, closing serial port.");
    serial_comm_.closePort();
}

void RelayNode::handleMessage(const std::vector<uint8_t> & data)
{
    auto msg = serial_relay::msg::SerialRawData();
    msg.data = data;
    serial_raw_data_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "publish data %ld bytes", data.size());
}
