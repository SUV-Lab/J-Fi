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

  port_name_ = this->get_parameter("port_name").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  system_id_ = static_cast<uint8_t>(this->get_parameter("system_id").as_int());
  component_id_ = static_cast<uint8_t>(this->get_parameter("component_id").as_int());

  RCLCPP_INFO(this->get_logger(),
              "Starting SerialCommNode with port: %s, baud_rate: %d, system_id: %d, component_id: %d",
              port_name_.c_str(), baud_rate_, system_id_, component_id_);

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

  std::string sid = std::to_string(system_id_);
  const std::string topic_prefix = "/V" + sid;

  // Create subscription for outgoing messages.
  sub_to_serial_poly_traj_ = this->create_subscription<path_manager::msg::PolyTraj>(
      topic_prefix + "/planning/broadcast_traj_send", qos,
      [this](const path_manager::msg::PolyTraj::SharedPtr msg)
      {
        auto modified_msg = std::make_shared<path_manager::msg::PolyTraj>(*msg);

        modified_msg->coef_z.clear();

        const int precision = 6;
        const size_t max_size = 12;
        std::vector<float> optimized_coef_x, optimized_coef_y;

        if (modified_msg->coef_x.size() != modified_msg->coef_y.size()) {
          RCLCPP_WARN(this->get_logger(), "Input coef_x size (%zu) differs from coef_y size (%zu)",
                      modified_msg->coef_x.size(), modified_msg->coef_y.size());
        }

        for (size_t i = 0; i < std::min(modified_msg->coef_x.size(), max_size); ++i) {
          optimized_coef_x.push_back(modified_msg->coef_x[i]);
          optimized_coef_y.push_back(modified_msg->coef_y[i]);
        }
        modified_msg->coef_x = optimized_coef_x;
        modified_msg->coef_y = optimized_coef_y;

        auto serialized_data = jfi_comm_.serialize_message(modified_msg);
        if (!serialized_data.empty())
        {
          jfi_comm_.send(TID_POLY_TRAJ, serialized_data);
          // RCLCPP_INFO(this->get_logger(), "Sent modified trajectory setpoint message via serial.");
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "[SerialCommNode] Failed to serialize modified trajectory setpoint message.");
        }
      });

  // Create publisher for incoming messages.
  pub_from_serial_poly_traj_ = this->create_publisher<path_manager::msg::PolyTraj>(
      topic_prefix + "/j_fi/broadcast_traj_recv", qos);
}

SerialCommNode::~SerialCommNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down SerialCommNode, closing serial port.");
  jfi_comm_.closePort();
}

void SerialCommNode::handleMessage(const int tid, const std::vector<uint8_t> & data)
{
  switch (tid) {
    case TID_POLY_TRAJ:
    {
      try {
        path_manager::msg::PolyTraj polytraj_msg = jfi_comm_.deserialize_message<path_manager::msg::PolyTraj>(data);
        if (polytraj_msg.coef_x.size() != polytraj_msg.coef_y.size()) {
          RCLCPP_WARN(this->get_logger(), "coef_x size (%zu) differs from coef_y size (%zu), using coef_x size",
                      polytraj_msg.coef_x.size(), polytraj_msg.coef_y.size());
        }
        size_t array_size = polytraj_msg.coef_x.size();
        polytraj_msg.coef_z = std::vector<float>(array_size, 0.0f);
        pub_from_serial_poly_traj_->publish(polytraj_msg);
        // RCLCPP_INFO(this->get_logger(), "Received and published TID_POLY_TRAJ message.");
      } catch (const std::exception & e) {
        // RCLCPP_ERROR(this->get_logger(), "[SerialCommNode] Failed to deserialize TID_POLY_TRAJ message: %s", e.what());
      }
    }
    break;
    default:
      RCLCPP_WARN(this->get_logger(), "[SerialCommNode] Received unknown message TID: %d", tid);
      break;
  }
}
