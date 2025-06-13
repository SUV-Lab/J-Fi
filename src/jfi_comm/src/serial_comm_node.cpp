#include "serial_comm_node.hpp"
#include "rclcpp/qos.hpp"
#include <functional>
#include <cstring>

SerialCommNode::SerialCommNode()
: Node("serial_comm_node")
{
  /* -------- Parameter handling ----------------------------------------- */
  declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  declare_parameter<int>("baud_rate", 115200);
  declare_parameter<int>("system_id", 1);
  declare_parameter<int>("component_id", 1);
  declare_parameter<std::vector<int>>("system_id_list", {1,2,3,4});
  
  port_name_      = get_parameter("port_name").as_string();
  baud_rate_      = get_parameter("baud_rate").as_int();
  system_id_      = static_cast<uint8_t>(get_parameter("system_id").as_int());
  component_id_   = static_cast<uint8_t>(get_parameter("component_id").as_int());
  system_id_list_ = get_parameter("system_id_list").as_integer_array();

  topic_prefix_jfi_ = "drone" + std::to_string(system_id_) + "/jfi/";

  RCLCPP_INFO(get_logger(),
              "SerialCommNode[%u] on %s @ %d bps",
              system_id_, port_name_.c_str(), baud_rate_);

  /* -------- JFiComm init ---------------------------------------------- */
  if (!jfi_comm_.init(
        std::bind(&SerialCommNode::handleMessage, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3),
        port_name_, baud_rate_, system_id_, component_id_))
  {
    RCLCPP_FATAL(get_logger(), "Failed to initialise JFiComm");
    throw std::runtime_error("JFiComm init failed");
  }

  /* -------- Subscriptions ---------------------------------------------- */
  sub_ranging_ = create_subscription<uwb_msgs::msg::Ranging>(
    topic_prefix_jfi_ + "in/ranging",
    rclcpp::SensorDataQoS(),
    [this](uwb_msgs::msg::Ranging::SharedPtr msg)
    {
      std::scoped_lock lk(cache_mtx_);
      latest_ranging_ = *msg;
    });

  sub_target_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    topic_prefix_jfi_ + "in/target",
    rclcpp::SensorDataQoS(),
    [this](px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
    {
      std::scoped_lock lk(cache_mtx_);
      latest_target_ = *msg;
    });

  /* -------- Publishers ------------------------------------------------- */
  for (int id : system_id_list_) {
    if (id == system_id_) continue;

    pub_ranging_map_[id] = create_publisher<uwb_msgs::msg::Ranging>(
      topic_prefix_jfi_ + "out/drone" + std::to_string(id) + "/ranging",
      rclcpp::SensorDataQoS());

    pub_target_map_[id] = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      topic_prefix_jfi_ + "out/drone" + std::to_string(id) + "/target",
      rclcpp::SensorDataQoS());
  }

  /* -------- 25 Hz batch timer ----------------------------------------- */
  timer_ = create_wall_timer(40ms, std::bind(&SerialCommNode::timerCallback, this));
}

SerialCommNode::~SerialCommNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down SerialCommNode");
  jfi_comm_.closePort();
}

/* ====================================================================== */
/*  TLV helper                                                            */
/* ====================================================================== */
void SerialCommNode::writeTLV(std::vector<uint8_t>& buf,
                              uint8_t tid,
                              const std::vector<uint8_t>& payload)
{
  buf.push_back(tid);                                     // T
  uint16_t len = static_cast<uint16_t>(payload.size());   // L
  const uint8_t* p = reinterpret_cast<uint8_t*>(&len);
  buf.insert(buf.end(), p, p + sizeof(len));              // L bytes
  buf.insert(buf.end(), payload.begin(), payload.end());  // V
}

/* ====================================================================== */
/* timer callback                                                         */
/* ====================================================================== */
void SerialCommNode::timerCallback()
{
  std::vector<uint8_t> batch;
  {
    std::scoped_lock lk(cache_mtx_);

    if (latest_ranging_) {
      auto bin = jfi_comm_.serialize_message(std::make_shared<uwb_msgs::msg::Ranging>(latest_ranging_.value()));
      writeTLV(batch, TID_RANGING, bin);
      // latest_ranging_.reset();
    }
    if (latest_target_) {
      auto bin = jfi_comm_.serialize_message(std::make_shared<px4_msgs::msg::TrajectorySetpoint>(latest_target_.value()));
      writeTLV(batch, TID_TRAJECTORY, bin);
      // latest_target_.reset();
    }
  }

  if (!batch.empty()) {
    jfi_comm_.send(TID_BATCH, batch);
    // RCLCPP_DEBUG(get_logger(), "Batch packet sent (%zu B)", batch.size());
  }
}

/* ====================================================================== */
/*  Incoming serial handler                                               */
/* ====================================================================== */
void SerialCommNode::handleMessage(int tid,
                                   uint8_t src_sysid,
                                   const std::vector<uint8_t>& data)
{
  switch (tid)
  {
    case TID_RANGING: {
      try {
        auto msg = jfi_comm_.deserialize_message<uwb_msgs::msg::Ranging>(data);
        auto it  = pub_ranging_map_.find(src_sysid);
        if (it != pub_ranging_map_.end())
          it->second->publish(msg);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Ranging deserialise failed: %s", e.what());
      }
      break;
    }

    case TID_TRAJECTORY: {
      try {
        auto msg = jfi_comm_.deserialize_message<px4_msgs::msg::TrajectorySetpoint>(data);
        auto it  = pub_target_map_.find(src_sysid);
        if (it != pub_target_map_.end())
          it->second->publish(msg);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Trajectory deserialise failed: %s", e.what());
      }
      break;
    }

    /* ------------ TLV batch container -------------------------------- */
    case TID_BATCH: {
      size_t i = 0;
      while (i + 3 <= data.size()) {               // at least tid + len(2)
        uint8_t  sub_tid = data[i++];
        uint16_t sub_len;
        std::memcpy(&sub_len, &data[i], sizeof(sub_len));
        i += sizeof(sub_len);
        if (i + sub_len > data.size()) {
          RCLCPP_WARN(get_logger(), "Corrupted batch packet dropped");
          break;
        }
        std::vector<uint8_t> slice(data.begin() + i,
                                   data.begin() + i + sub_len);
        i += sub_len;
        handleMessage(sub_tid, src_sysid, slice);  // recurse
      }
      break;
    }

    default:
      RCLCPP_WARN(get_logger(), "Unknown TID %d from sysid %u", tid, src_sysid);
  }
}
