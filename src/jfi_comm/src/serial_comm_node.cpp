#include "serial_comm_node.hpp"
#include <functional>

SerialCommNode::SerialCommNode()
: Node("serial_comm_node")
{
  // Declare and get parameters.
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<int>("system_id", 1);
  this->declare_parameter<int>("component_id", 1);
  this->declare_parameter<std::string>("mode", "sender");  // Mode parameter: "sender" or "receiver"

  port_name_ = this->get_parameter("port_name").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  system_id_ = static_cast<uint8_t>(this->get_parameter("system_id").as_int());
  component_id_ = static_cast<uint8_t>(this->get_parameter("component_id").as_int());
  mode_ = this->get_parameter("mode").as_string();

  RCLCPP_INFO(this->get_logger(),
              "Starting SerialCommNode with port: %s, baud_rate: %d, system_id: %d, component_id: %d, mode: %s",
              port_name_.c_str(), baud_rate_, system_id_, component_id_, mode_.c_str());

  // Initialize JFiComm.
  if (!jfi_comm_.init(
        std::bind(&SerialCommNode::handleMessage, this, std::placeholders::_1, std::placeholders::_2),
        port_name_, baud_rate_, system_id_, component_id_)) {
    RCLCPP_ERROR(this->get_logger(), "[SerialCommNode] Failed to initialize JFiComm on port: %s", port_name_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "JFiComm initialized successfully.");
  }

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  const std::string topic_prefix = "vehicle" + std::to_string(system_id_);

  // Conditional pub/sub based on mode_
  if (mode_ == "sender") {
    // Create subscription for outgoing messages.
    sub_to_serial_trajectory_setpoint_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "leader/to_serial_trajectory_setpoint", qos,
      [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        auto serialized_data = jfi_comm_.serialize_message(msg);
        if (!serialized_data.empty()) {
          jfi_comm_.send(TID_TRAJECTORY_SETPOINT, serialized_data);
          RCLCPP_INFO(this->get_logger(), "Sent trajectory setpoint message via serial.");
        } else {
          RCLCPP_WARN(this->get_logger(), "[SerialCommNode] Failed to serialize trajectory setpoint message for serial transmission.");
        }
      }
    );
  } else {
    // Create publisher for incoming messages.
    pub_from_serial_trajectory_setpoint_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      topic_prefix + "/from_serial_trajectory_setpoint", qos);
  }
}

SerialCommNode::~SerialCommNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down SerialCommNode, closing serial port.");
  jfi_comm_.closePort();
}

void SerialCommNode::handleMessage(const int tid, const std::vector<uint8_t> & data)
{
  if (mode_ != "receiver") {
    return;  // Only process incoming serial data in receiver mode
  }

  switch (tid) {
    case TID_TRAJECTORY_SETPOINT:
    {
      try {
        px4_msgs::msg::TrajectorySetpoint trajectory_msg = jfi_comm_.deserialize_message<px4_msgs::msg::TrajectorySetpoint>(data);
        pub_from_serial_trajectory_setpoint_->publish(trajectory_msg);
        RCLCPP_INFO(this->get_logger(), "Received and published TID_TRAJECTORY_SETPOINT message.");
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "[SerialCommNode] Failed to deserialize TID_TRAJECTORY_SETPOINT message: %s", e.what());
      }
    }
    break;
    default:
      RCLCPP_WARN(this->get_logger(), "[SerialCommNode] Received unknown message TID: %d", tid);
      break;
  }
}
