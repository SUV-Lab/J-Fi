#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include "jfi_comm/msg/swarm_comm.hpp"

#include <map>
#include <numeric>
#include <vector>

using namespace std::chrono_literals;

struct PeerStats {
    uint32_t last_rx_app_seq;
    bool last_rx_app_seq_known = false;
    size_t packets_received_in_period = 0;
    size_t lost_packets_in_period = 0;
    std::vector<double> latency_buffer;
};

template <typename T>
class Ros2Serializer
{
public:
    std::vector<uint8_t> serialize(const std::shared_ptr<T>& msg)
    {
        rclcpp::Serialization<T> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(msg.get(), &serialized_msg);
        std::vector<uint8_t> data(
            serialized_msg.get_rcl_serialized_message().buffer,
            serialized_msg.get_rcl_serialized_message().buffer +
            serialized_msg.get_rcl_serialized_message().buffer_length);
        return data;
    }

    T deserialize(const std::vector<uint8_t>& data)
    {
        T msg;
        rclcpp::Serialization<T> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serialized_msg.reserve(data.size());
        serialized_msg.get_rcl_serialized_message().buffer_length = data.size();
        std::memcpy(
            serialized_msg.get_rcl_serialized_message().buffer,
            data.data(),
            data.size());
        serializer.deserialize_message(&serialized_msg, &msg);
        return msg;
    }
};

class EvaluatorNode : public rclcpp::Node
{
public:
  EvaluatorNode() : Node("evaluator_node"), tx_seq_(0), last_rx_seq_known_(false)
  {
    publisher_ = this->create_publisher<jfi_comm::msg::SwarmComm>("jfi_comm/in/packet", 10);
    subscription_ = this->create_subscription<jfi_comm::msg::SwarmComm>(
        "jfi_comm/out/packet", 10,
        std::bind(&EvaluatorNode::packet_callback, this, std::placeholders::_1));

    send_timer_ = this->create_wall_timer(100ms, std::bind(&EvaluatorNode::send_packet, this)); // 10Hz
    report_timer_ = this->create_wall_timer(5s, std::bind(&EvaluatorNode::report_callback, this));

    RCLCPP_INFO(this->get_logger(), "Evaluator node started.");
  }

private:
  void send_packet()
  {
    auto traj_msg = std::make_shared<trajectory_msgs::msg::MultiDOFJointTrajectory>();
    traj_msg->header.stamp = this->get_clock()->now();
    // traj_msg->header.frame_id = std::to_string(tx_seq_++);
    traj_msg->joint_names = {"joint1", "joint2"};
    for (int i = 0; i < 2; ++i) {
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        point.transforms.resize(1);
        point.transforms[0].translation.x = 1.0 + i;
        point.velocities.resize(1);
        point.velocities[0].linear.x = 0.1 * i;
        point.accelerations.resize(1);
        point.accelerations[0].linear.x = 0.01 * i;
        traj_msg->points.push_back(point);
    }

    auto serialized_data = serializer_.serialize(traj_msg);
    // total_tx_bytes_in_period_ += serialized_data.size();

    auto packet = std::make_unique<jfi_comm::msg::SwarmComm>();
    packet->app_seq = tx_seq_++;
    packet->tid = 2; // TID_TRAJECTORY
    packet->payload = serialized_data;
    publisher_->publish(std::move(packet));
    
    packets_sent_in_period_++;
  }

  void packet_callback(const jfi_comm::msg::SwarmComm::SharedPtr msg)
  {
    uint8_t peer_id = msg->src_sysid;
    PeerStats& stats = peer_statistics_[peer_id];
    stats.packets_received_in_period++;

    if (stats.last_rx_app_seq_known) {
      if (msg->app_seq > stats.last_rx_app_seq + 1) {
            stats.lost_packets_in_period += (msg->app_seq - stats.last_rx_app_seq - 1);
        }
    }
    stats.last_rx_app_seq = msg->app_seq;
    stats.last_rx_app_seq_known = true;

    try {
      auto traj_msg = serializer_.deserialize(msg->payload);
      rclcpp::Time start_time = traj_msg.header.stamp;
      rclcpp::Duration latency = this->get_clock()->now() - start_time;
      stats.latency_buffer.push_back(latency.seconds() * 1000.0);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Payload from system %u deserialization failed: %s", peer_id, e.what());
    }
  }

  void report_callback()
  {
    RCLCPP_INFO(this->get_logger(), "========== N:N COMMS REPORT (5s Interval) ==========");

    // double avg_tx_size = 0.0;
    // if (packets_sent_in_period_ > 0) {
    //     avg_tx_size = static_cast<double>(total_tx_bytes_in_period_) / packets_sent_in_period_;
    // }
    // RCLCPP_INFO(this->get_logger(), "TX (Sent): %zu packets (Avg size: %.1f bytes)", 
    //           packets_sent_in_period_, avg_tx_size);

    RCLCPP_INFO(this->get_logger(), "TX (Sent): %zu packets", packets_sent_in_period_);

    for (auto const& [peer_id, stats] : peer_statistics_) {
      double avg_latency = 0.0, max_latency = 0.0;
      if (!stats.latency_buffer.empty()) {
        double sum = std::accumulate(stats.latency_buffer.begin(), stats.latency_buffer.end(), 0.0);
        avg_latency = sum / stats.latency_buffer.size();
        max_latency = *std::max_element(stats.latency_buffer.begin(), stats.latency_buffer.end());
      }
      
      size_t total_expected = stats.packets_received_in_period + stats.lost_packets_in_period;
      double loss_rate = (total_expected > 0) ? (static_cast<double>(stats.lost_packets_in_period) / total_expected * 100.0) : 0.0;
      
      RCLCPP_INFO(this->get_logger(), "--- RX Stats from Peer ID: %u ---", peer_id);
      RCLCPP_INFO(this->get_logger(), "    Received: %zu | Lost: %zu (%.2f%% loss)",
                  stats.packets_received_in_period, stats.lost_packets_in_period, loss_rate);
      if (!stats.latency_buffer.empty()) {
        RCLCPP_INFO(this->get_logger(), "    One-way Latency(ms): Avg: %.2f | Max: %.2f", avg_latency, max_latency);
      }
    }
    RCLCPP_INFO(this->get_logger(), "======================================================");

    packets_sent_in_period_ = 0;
    // total_tx_bytes_in_period_ = 0;
    peer_statistics_.clear();
  }

  Ros2Serializer<trajectory_msgs::msg::MultiDOFJointTrajectory> serializer_;
  rclcpp::Publisher<jfi_comm::msg::SwarmComm>::SharedPtr publisher_;
  rclcpp::Subscription<jfi_comm::msg::SwarmComm>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  
  uint32_t tx_seq_;
  uint8_t last_rx_seq_;
  bool last_rx_seq_known_;
  size_t packets_sent_in_period_ = 0;
  // size_t total_tx_bytes_in_period_ = 0;

  std::map<uint8_t, PeerStats> peer_statistics_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvaluatorNode>());
  rclcpp::shutdown();
  return 0;
}