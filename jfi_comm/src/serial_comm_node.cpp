#include "serial_comm_node.hpp"
#include <functional>

SerialCommNode::SerialCommNode()
: Node("serial_comm_node_example")
{
  /* -------- 1. Parameter handling ------------------------------------- */
  declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  declare_parameter<int>("baud_rate", 115200);
  declare_parameter<int>("system_id", 1);
  declare_parameter<int>("component_id", 1);
  
  port_name_    = get_parameter("port_name").as_string();
  baud_rate_    = get_parameter("baud_rate").as_int();
  system_id_    = static_cast<uint8_t>(get_parameter("system_id").as_int());
  component_id_ = static_cast<uint8_t>(get_parameter("component_id").as_int());

  RCLCPP_INFO(get_logger(),
              "Starting example node for JFiComm with sysid %u on %s @ %d bps",
              system_id_, port_name_.c_str(), baud_rate_);

  /* -------- 2. JFiComm initialization --------------------------------- */
  if (!jfi_comm_.init(
        std::bind(&SerialCommNode::handleMessage, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
        port_name_, baud_rate_, system_id_, component_id_))
  {
    RCLCPP_FATAL(get_logger(), "Failed to initialise JFiComm. Shutting down.");
    throw std::runtime_error("JFiComm init failed");
  }

  /* -------- 3. ROS Publishers (Serial -> ROS) ------------------------- */
  // Publisher for incoming serial packets. Change the topic name and message type as needed.
  pub_packet_ = this->create_publisher<jfi_comm::msg::SwarmComm>("jfi_comm/out/packet", 10);

  /* -------- 4. ROS Subscribers (ROS -> Serial) ------------------------ */
  // Subscriber for outgoing serial packets. Change the topic name and message type as needed.
  sub_packet_ = this->create_subscription<jfi_comm::msg::SwarmComm>(
    "jfi_comm/in/packet", 10,
    [this](const jfi_comm::msg::SwarmComm::SharedPtr msg) {
      jfi_comm_.send(msg->tid, msg->payload);
    });
}

SerialCommNode::~SerialCommNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down SerialCommNode");
  jfi_comm_.closePort();
}

/**
 * @brief This is the main callback that handles all data received from the serial port.
 */
void SerialCommNode::handleMessage(uint8_t seq, uint8_t tid,
                                   uint8_t src_sysid,
                                   const std::vector<uint8_t>& data)
{
  RCLCPP_DEBUG(get_logger(), "Received message with TID %u from source system %u", tid, src_sysid);

  auto packet_msg = std::make_unique<jfi_comm::msg::SwarmComm>();

  packet_msg->header.stamp = this->get_clock()->now();
  packet_msg->src_sysid = src_sysid;
  packet_msg->seq = seq;
  packet_msg->tid = tid;
  packet_msg->payload = data;

  pub_packet_->publish(std::move(packet_msg));
}