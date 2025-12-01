#include "serial_comm_node.hpp"
#include <functional>
#include <algorithm>
#include <cstring>

SerialCommNode::SerialCommNode()
: Node("serial_comm_node"),
  last_sent_formation_cmd_sequence_(-1),
  last_polytraj_send_time_(std::chrono::steady_clock::now())
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
        const int precision = 3;  // Reduced from 6 to 3 (1mm accuracy -> 1cm accuracy) to reduce packet size
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

        // Remove trailing zeros from coef arrays to reduce packet size
        // Polynomial coefficients often have many zeros at the end
        auto remove_trailing_zeros = [](std::vector<float>& vec) {
            while (!vec.empty() && std::abs(vec.back()) < 1e-9f) {
                vec.pop_back();
            }
        };
        remove_trailing_zeros(modified_msg->coef_x);
        remove_trailing_zeros(modified_msg->coef_y);

        // ===== Quantization Compression =====
        // Convert float32 arrays to uint8 with min/max metadata for 75% size reduction

        auto quantize_array = [](const std::vector<float>& input, std::vector<uint8_t>& output, float& min_val, float& max_val) {
            if (input.empty()) {
                min_val = 0.0f;
                max_val = 0.0f;
                return;
            }

            min_val = *std::min_element(input.begin(), input.end());
            max_val = *std::max_element(input.begin(), input.end());

            output.resize(input.size());
            float range = max_val - min_val;

            if (range < 1e-9f) {
                // All values are the same
                std::fill(output.begin(), output.end(), 0);
            } else {
                for (size_t i = 0; i < input.size(); ++i) {
                    float normalized = (input[i] - min_val) / range;
                    output[i] = static_cast<uint8_t>(std::clamp(normalized * 255.0f, 0.0f, 255.0f));
                }
            }
        };

        std::vector<uint8_t> coef_x_quantized, coef_y_quantized, duration_quantized;
        float coef_x_min, coef_x_max, coef_y_min, coef_y_max, duration_min, duration_max;

        quantize_array(modified_msg->coef_x, coef_x_quantized, coef_x_min, coef_x_max);
        quantize_array(modified_msg->coef_y, coef_y_quantized, coef_y_min, coef_y_max);
        quantize_array(modified_msg->duration, duration_quantized, duration_min, duration_max);

        // Pack compressed data into byte array
        // Format: [header(14)] [coef_x_quantized] [coef_y_quantized] [duration_quantized]
        // Header: drone_id(2) traj_id(4) start_time(8) order(1)
        //         coef_x: size(2) min(4) max(4)
        //         coef_y: size(2) min(4) max(4)
        //         duration: size(2) min(4) max(4)

        size_t header_size = 2 + 4 + 8 + 1 + (2+4+4)*3;  // 45 bytes
        size_t total_size = header_size + coef_x_quantized.size() + coef_y_quantized.size() + duration_quantized.size();

        std::vector<uint8_t> serialized_data(total_size);
        size_t offset = 0;

        // drone_id (int16)
        *reinterpret_cast<int16_t*>(&serialized_data[offset]) = modified_msg->drone_id;
        offset += 2;

        // traj_id (int32)
        *reinterpret_cast<int32_t*>(&serialized_data[offset]) = modified_msg->traj_id;
        offset += 4;

        // start_time (2x int32 = sec + nanosec)
        *reinterpret_cast<int32_t*>(&serialized_data[offset]) = modified_msg->start_time.sec;
        offset += 4;
        *reinterpret_cast<uint32_t*>(&serialized_data[offset]) = modified_msg->start_time.nanosec;
        offset += 4;

        // order (uint8)
        serialized_data[offset++] = modified_msg->order;

        // coef_x metadata
        *reinterpret_cast<uint16_t*>(&serialized_data[offset]) = static_cast<uint16_t>(coef_x_quantized.size());
        offset += 2;
        *reinterpret_cast<float*>(&serialized_data[offset]) = coef_x_min;
        offset += 4;
        *reinterpret_cast<float*>(&serialized_data[offset]) = coef_x_max;
        offset += 4;

        // coef_y metadata
        *reinterpret_cast<uint16_t*>(&serialized_data[offset]) = static_cast<uint16_t>(coef_y_quantized.size());
        offset += 2;
        *reinterpret_cast<float*>(&serialized_data[offset]) = coef_y_min;
        offset += 4;
        *reinterpret_cast<float*>(&serialized_data[offset]) = coef_y_max;
        offset += 4;

        // duration metadata
        *reinterpret_cast<uint16_t*>(&serialized_data[offset]) = static_cast<uint16_t>(duration_quantized.size());
        offset += 2;
        *reinterpret_cast<float*>(&serialized_data[offset]) = duration_min;
        offset += 4;
        *reinterpret_cast<float*>(&serialized_data[offset]) = duration_max;
        offset += 4;

        // Copy quantized data
        std::memcpy(&serialized_data[offset], coef_x_quantized.data(), coef_x_quantized.size());
        offset += coef_x_quantized.size();
        std::memcpy(&serialized_data[offset], coef_y_quantized.data(), coef_y_quantized.size());
        offset += coef_y_quantized.size();
        std::memcpy(&serialized_data[offset], duration_quantized.data(), duration_quantized.size());

        // ===== End Quantization Compression =====
        if (!serialized_data.empty())
        {
          jfi_comm_.send(TID_POLY_TRAJ, serialized_data);
          RCLCPP_INFO(this->get_logger(), "Sent PolyTraj: drone_id=%d, order=%d, coef_x=%zu, coef_y=%zu, duration=%zu, size=%zu bytes",
                      modified_msg->drone_id, modified_msg->order,
                      modified_msg->coef_x.size(), modified_msg->coef_y.size(), modified_msg->duration.size(),
                      serialized_data.size());
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "[SerialCommNode] Failed to serialize modified trajectory setpoint message.");
        }
      });

  // Create publisher for incoming messages.
  pub_from_serial_poly_traj_ = this->create_publisher<path_manager::msg::PolyTraj>(
      topic_prefix + "/j_fi/broadcast_traj_recv", qos);

  // FormationCommand subscription (send to serial)
  // ONLY Commander drone (system_id == 1) subscribes to Commander's topic
  if (system_id_ == 1) {
    sub_to_serial_formation_cmd_ = this->create_subscription<path_manager::msg::FormationCommand>(
        "formation_command", qos,
        [this](const path_manager::msg::FormationCommand::SharedPtr msg)
        {
          // Only send if sequence is new (prevent duplicate transmissions)
          if (msg->sequence == last_sent_formation_cmd_sequence_) {
            // Skip duplicate - Commander republishes at 1Hz for robustness
            return;
          }

          auto serialized_data = jfi_comm_.serialize_message(msg);
          if (!serialized_data.empty())
          {
            jfi_comm_.send(TID_FORMATION_COMMAND, serialized_data);
            last_sent_formation_cmd_sequence_ = msg->sequence;
            RCLCPP_INFO(this->get_logger(), "Sent FormationCommand via serial: seq=%d, mission=%s->%s, formation=%s, waypoints=%zu, serialized_size=%zu",
                        msg->sequence, msg->current_mission_id.c_str(), msg->next_mission_id.c_str(),
                        msg->formation_type.c_str(), msg->waypoints.size(), serialized_data.size());
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "[SerialCommNode] Failed to serialize FormationCommand message.");
          }
        });
    RCLCPP_INFO(this->get_logger(), "Commander mode: Subscribed to 'formation_command' for serial transmission");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Follower mode (system_id=%d): Not subscribing to formation_command", system_id_);
  }

  // FormationCommand publisher (receive from serial)
  // ALL drones publish received FormationCommand from serial
  // Use different topic name to avoid conflict with Commander's direct publish
  pub_from_serial_formation_cmd_ = this->create_publisher<path_manager::msg::FormationCommand>(
      "formation_command_serial", qos);
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
        // ===== Quantization Decompression =====
        // Decompress uint8 quantized data back to float32 arrays

        if (data.size() < 45) {
          RCLCPP_ERROR(this->get_logger(), "Received PolyTraj data too small: %zu bytes", data.size());
          break;
        }

        path_manager::msg::PolyTraj polytraj_msg;
        size_t offset = 0;

        // Parse header
        polytraj_msg.drone_id = *reinterpret_cast<const int16_t*>(&data[offset]);
        offset += 2;

        polytraj_msg.traj_id = *reinterpret_cast<const int32_t*>(&data[offset]);
        offset += 4;

        polytraj_msg.start_time.sec = *reinterpret_cast<const int32_t*>(&data[offset]);
        offset += 4;
        polytraj_msg.start_time.nanosec = *reinterpret_cast<const uint32_t*>(&data[offset]);
        offset += 4;

        polytraj_msg.order = data[offset++];

        // Parse coef_x metadata
        uint16_t coef_x_size = *reinterpret_cast<const uint16_t*>(&data[offset]);
        offset += 2;
        float coef_x_min = *reinterpret_cast<const float*>(&data[offset]);
        offset += 4;
        float coef_x_max = *reinterpret_cast<const float*>(&data[offset]);
        offset += 4;

        // Parse coef_y metadata
        uint16_t coef_y_size = *reinterpret_cast<const uint16_t*>(&data[offset]);
        offset += 2;
        float coef_y_min = *reinterpret_cast<const float*>(&data[offset]);
        offset += 4;
        float coef_y_max = *reinterpret_cast<const float*>(&data[offset]);
        offset += 4;

        // Parse duration metadata
        uint16_t duration_size = *reinterpret_cast<const uint16_t*>(&data[offset]);
        offset += 2;
        float duration_min = *reinterpret_cast<const float*>(&data[offset]);
        offset += 4;
        float duration_max = *reinterpret_cast<const float*>(&data[offset]);
        offset += 4;

        // Dequantize arrays
        auto dequantize_array = [](const uint8_t* quantized, size_t size, float min_val, float max_val, std::vector<float>& output) {
            output.resize(size);
            float range = max_val - min_val;

            if (range < 1e-9f) {
                std::fill(output.begin(), output.end(), min_val);
            } else {
                for (size_t i = 0; i < size; ++i) {
                    float normalized = quantized[i] / 255.0f;
                    output[i] = min_val + normalized * range;
                }
            }
        };

        dequantize_array(&data[offset], coef_x_size, coef_x_min, coef_x_max, polytraj_msg.coef_x);
        offset += coef_x_size;

        dequantize_array(&data[offset], coef_y_size, coef_y_min, coef_y_max, polytraj_msg.coef_y);
        offset += coef_y_size;

        dequantize_array(&data[offset], duration_size, duration_min, duration_max, polytraj_msg.duration);

        RCLCPP_INFO(this->get_logger(), "Received PolyTraj (quantized): drone_id=%d, coef_x=%zu, coef_y=%zu, duration=%zu, compressed_size=%zu bytes",
                    polytraj_msg.drone_id, polytraj_msg.coef_x.size(), polytraj_msg.coef_y.size(), polytraj_msg.duration.size(), data.size());

        // ===== End Quantization Decompression =====
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
    case TID_FORMATION_COMMAND:
    {
      try {
        path_manager::msg::FormationCommand formation_cmd_msg =
            jfi_comm_.deserialize_message<path_manager::msg::FormationCommand>(data);

        pub_from_serial_formation_cmd_->publish(formation_cmd_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Received and published FormationCommand: seq=%d, mission=%s->%s, formation=%s, waypoints=%zu",
                    formation_cmd_msg.sequence,
                    formation_cmd_msg.current_mission_id.c_str(),
                    formation_cmd_msg.next_mission_id.c_str(),
                    formation_cmd_msg.formation_type.c_str(),
                    formation_cmd_msg.waypoints.size());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(),
                     "[SerialCommNode] Failed to deserialize TID_FORMATION_COMMAND message: %s", e.what());
      }
    }
    break;
    default:
      RCLCPP_WARN(this->get_logger(), "[SerialCommNode] Received unknown message TID: %d", tid);
      break;
  }
}

