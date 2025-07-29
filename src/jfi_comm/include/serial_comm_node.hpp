#ifndef SERIAL_COMM_NODE_HPP
#define SERIAL_COMM_NODE_HPP

#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// #include <uwb_msgs/msg/ranging.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "jfi_comm.hpp"

using namespace std::chrono_literals;

class SerialCommNode : public rclcpp::Node
{
public:
  enum TID : uint8_t
  {
    // TID_RANGING    = 1,
    TID_TRAJECTORY = 2,
    TID_BATCH      = 99          ///< container for TLV-bundled messages
  };

  SerialCommNode();
  ~SerialCommNode();

private:
  /* ---------- Callbacks -------------------------------------------------- */
  void handleMessage(int tid,
                     uint8_t src_sysid,
                     const std::vector<uint8_t>& data);

  void timerCallback();                   ///< 25 Hz flush-and-send routine
  static void writeTLV(std::vector<uint8_t>& buf,
                       uint8_t tid,
                       const std::vector<uint8_t>& payload);

  /* ---------- Members ---------------------------------------------------- */
  JFiComm jfi_comm_;

  // Serial port parameters
  std::string port_name_;
  int         baud_rate_;

  // Local MAVLink identity
  uint8_t system_id_;
  uint8_t component_id_;

  // ROS topic prefix   (e.g. "drone1/jfi/")
  std::string topic_prefix_jfi_;

  // All drone IDs in the swarm
  std::vector<int64_t> system_id_list_;

  /* ROS interfaces -------------------------------------------------------- */
  // Subscriptions (ROS → serial)
  // rclcpp::Subscription<uwb_msgs::msg::Ranging>::SharedPtr            sub_ranging_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_target_;

  // Publishers   (serial → ROS)
  // std::map<int, rclcpp::Publisher<uwb_msgs::msg::Ranging>::SharedPtr>            pub_ranging_map_;
  std::map<int, rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr> pub_target_map_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  /* Cached latest messages ------------------------------------------------ */
  std::mutex cache_mtx_;
  // std::optional<uwb_msgs::msg::Ranging>            latest_ranging_;
  std::optional<px4_msgs::msg::TrajectorySetpoint> latest_target_;
};

#endif  // SERIAL_COMM_NODE_HPP