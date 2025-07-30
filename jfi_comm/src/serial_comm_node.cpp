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

        const float threshold = 1e-10f;
        const int precision = 6;
        const float scale = std::pow(10.0f, static_cast<float>(precision));

        for (auto& coef : modified_msg->coef_x) {
            if (std::abs(coef) < threshold) {
                coef = 0.0f;
            } else {
                coef = std::round(coef * scale) / scale;
            }
        }
        for (auto& coef : modified_msg->coef_y) {
            if (std::abs(coef) < threshold) {
                coef = 0.0f;
            } else {
                coef = std::round(coef * scale) / scale;
            }
        }
        for (auto& dur : modified_msg->duration) {
            dur = std::round(dur * scale) / scale;
        }

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

        size_t target_size = polytraj_msg.duration.size() * 6;

        std::vector<float> new_coef_x;
        new_coef_x.reserve(target_size);
        size_t current_size = polytraj_msg.coef_x.size();
        for (size_t i = 0; i < std::min(current_size, target_size); ++i) {
          new_coef_x.push_back(polytraj_msg.coef_x[i]);
        }
        if (current_size > 0 && current_size < target_size) {
          float last_value = polytraj_msg.coef_x[current_size - 1];
          for (size_t i = current_size; i < target_size; ++i) {
            new_coef_x.push_back(last_value);
          }
        } else if (current_size == 0) {
          new_coef_x = std::vector<float>(target_size, 0.0f);
        }
        polytraj_msg.coef_x = new_coef_x;

        std::vector<float> new_coef_y;
        new_coef_y.reserve(target_size);
        current_size = polytraj_msg.coef_y.size();
        for (size_t i = 0; i < std::min(current_size, target_size); ++i) {
          new_coef_y.push_back(polytraj_msg.coef_y[i]);
        }
        if (current_size > 0 && current_size < target_size) {
          float last_value = polytraj_msg.coef_y[current_size - 1];
          for (size_t i = current_size; i < target_size; ++i) {
            new_coef_y.push_back(last_value);
          }
        } else if (current_size == 0) {
          new_coef_y = std::vector<float>(target_size, 0.0f);
        }
        polytraj_msg.coef_y = new_coef_y;
        polytraj_msg.coef_z = std::vector<float>(target_size, 0.0f);

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
