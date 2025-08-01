#include "serial_comm_node.hpp"
#include "rclcpp/qos.hpp"
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
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        port_name_, baud_rate_, system_id_, component_id_))
  {
    RCLCPP_FATAL(get_logger(), "Failed to initialise JFiComm. Shutting down.");
    throw std::runtime_error("JFiComm init failed");
  }

  /* -------- 3. ROS Publishers (Serial -> ROS) ------------------------- */
  // Create publishers for messages received FROM the serial port
  pub_string_ = create_publisher<std_msgs::msg::String>("jfi_comm/in/string", 10);

  /* -------- 4. ROS Subscribers (ROS -> Serial) ------------------------ */
  // Create subscribers for messages send TO the serial port
  sub_string_ = create_subscription<std_msgs::msg::String>(
    "jfi_comm/out/string", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(get_logger(), "Received String on topic, sending to serial...");
      auto serialized_data = jfi_comm_.serialize_message(msg);
      jfi_comm_.send(TID_ROS_STRING, serialized_data);
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
void SerialCommNode::handleMessage(uint8_t tid,
                                   uint8_t src_sysid,
                                   const std::vector<uint8_t>& data)
{
  RCLCPP_INFO(get_logger(), "Received message with TID %u from source system %u", tid, src_sysid);

  switch (tid)
  {
    case TID_ROS_STRING: {
      try {
        // Deserialize the byte vector back into a ROS message
        auto msg = jfi_comm_.deserialize_message<std_msgs::msg::String>(data);
        RCLCPP_INFO(get_logger(), "  -> Deserialized String: '%s'", msg.data.c_str());
        // Publish the message to a ROS topic
        pub_string_->publish(msg);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "String deserialization failed: %s", e.what());
      }
      break;
    }

    default:
      RCLCPP_WARN(get_logger(), "Received message with unknown TID: %d", tid);
  }
}