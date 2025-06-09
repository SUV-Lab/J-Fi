#include "serial_comm_node.hpp"
#include "rclcpp/qos.hpp"
#include <functional>

SerialCommNode::SerialCommNode()
: Node("serial_comm_node")
{
  // Declare and get parameters.
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<int>("system_id", 1);
  this->declare_parameter<int>("component_id", 1);
  this->declare_parameter<std::vector<int>>("system_id_list", {1,2,3,4});
  
  port_name_       = this->get_parameter("port_name").as_string();
  baud_rate_       = this->get_parameter("baud_rate").as_int();
  system_id_       = static_cast<uint8_t>(this->get_parameter("system_id").as_int());
  component_id_    = static_cast<uint8_t>(this->get_parameter("component_id").as_int());
  system_id_list_  = this->get_parameter("system_id_list").as_integer_array();

  // JFI topic prefix
  topic_prefix_jfi_ = "drone" + std::to_string(system_id_) + "/jfi/";

  RCLCPP_INFO(get_logger(),
    "SerialCommNode[%u] on %s @ %dbps",
    system_id_, port_name_.c_str(), baud_rate_);

  // Initialize JFiComm.
  if (!jfi_comm_.init(
        std::bind(&SerialCommNode::handleMessage, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3),
        port_name_, baud_rate_, system_id_, component_id_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to init JFiComm");
  }

  // Create subscriptions for outgoing messages.
  sub_to_serial_ranging_ = create_subscription<uwb_msgs::msg::Ranging>(
    topic_prefix_jfi_ + "in/ranging",
    rclcpp::SensorDataQoS(),
    [this](uwb_msgs::msg::Ranging::SharedPtr msg) {
      auto data = jfi_comm_.serialize_message(msg);
      if (!data.empty()) {
        jfi_comm_.send(TID_RANGING, data);
      }
    });

  sub_to_serial_trajectory_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    topic_prefix_jfi_ + "in/target",
    rclcpp::SensorDataQoS(),
    [this](px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
      auto data = jfi_comm_.serialize_message(msg);
      if (!data.empty()) {
        jfi_comm_.send(TID_TRAJECTORY, data);
      }
    });

  // Create publishers for incoming messages.
  for (int id : system_id_list_) {
    if (id == system_id_) continue;
    // Ranging
    auto r_pub = create_publisher<uwb_msgs::msg::Ranging>(
      topic_prefix_jfi_ + "out/drone" + std::to_string(id) + "/ranging", rclcpp::SensorDataQoS());
    pub_ranging_map_[id] = r_pub;

    // TrajectorySetpoint
    auto t_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      topic_prefix_jfi_ + "out/drone" + std::to_string(id) + "/target", rclcpp::SensorDataQoS());
    pub_trajectory_map_[id] = t_pub;
  }
}

SerialCommNode::~SerialCommNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down SerialCommNode");
  jfi_comm_.closePort();
}

void SerialCommNode::handleMessage(int tid, uint8_t src_sysid, const std::vector<uint8_t>& data)
{
  switch (tid) {
    case TID_RANGING: {
      try {
        auto msg = jfi_comm_.deserialize_message<uwb_msgs::msg::Ranging>(data);
        auto it  = pub_ranging_map_.find(src_sysid);
        if (it != pub_ranging_map_.end()) {
          it->second->publish(msg);
        } else {
          RCLCPP_WARN(get_logger(), "No publisher for Ranging from drone%u", src_sysid);
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Ranging deserialize failed: %s", e.what());
      }
      break;
    }
    case TID_TRAJECTORY: {
      try {
        auto msg = jfi_comm_.deserialize_message<px4_msgs::msg::TrajectorySetpoint>(data);
        auto it  = pub_trajectory_map_.find(src_sysid);
        if (it != pub_trajectory_map_.end()) {
          it->second->publish(msg);
        } else {
          RCLCPP_WARN(get_logger(), "No publisher for Target from drone%u", src_sysid);
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Target deserialize failed: %s", e.what());
      }
      break;
    }
    default:
      RCLCPP_WARN(get_logger(), "Unknown TID: %d from %u", tid, src_sysid);
  }
}
