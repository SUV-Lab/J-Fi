#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include "jfi_comm/msg/swarm_comm.hpp"

#include <map>
#include <numeric>
#include <vector>

using namespace std::chrono_literals;

// trajectory 메시지를 직렬화/역직렬화하기 위한 헬퍼 클래스
// (jfi_comm 라이브러리에서 가져옴)
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
    // comm_node로 SwarmComm 패킷을 보내는 퍼블리셔
    publisher_ = this->create_publisher<jfi_comm::msg::SwarmComm>("jfi_comm/in/packet", 10);

    // comm_node로부터 SwarmComm 패킷을 받는 서브스크라이버
    subscription_ = this->create_subscription<jfi_comm::msg::SwarmComm>(
        "jfi_comm/out/packet", 10,
        std::bind(&EvaluatorNode::packet_callback, this, std::placeholders::_1));

    // 100ms마다 (초당 10회) 메시지를 보내는 타이머
    send_timer_ = this->create_wall_timer(
        100ms, std::bind(&EvaluatorNode::send_packet, this));

    // 5초마다 통계 결과를 출력하는 타이머
    report_timer_ = this->create_wall_timer(
        5s, std::bind(&EvaluatorNode::report_callback, this));

    RCLCPP_INFO(this->get_logger(), "Bidirectional evaluator node started.");
  }

private:
  // 송신 로직
  void send_packet()
  {
    // 1. 평가용 Trajectory 메시지 생성
    auto traj_msg = std::make_shared<trajectory_msgs::msg::MultiDOFJointTrajectory>();
    // 애플리케이션 레벨의 타임스탬프와 순번을 Trajectory 헤더에 기록
    traj_msg->header.stamp = this->get_clock()->now();
    traj_msg->header.frame_id = std::to_string(tx_seq_++);
    traj_msg->joint_names = {"joint1", "joint2"};
    for (int i = 0; i < 2; ++i) {
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        point.transforms.resize(1);
        point.transforms[0].translation.x = 1.0 + i;
        traj_msg->points.push_back(point);
    }

    // 2. Trajectory 메시지를 바이트 벡터로 직렬화
    auto serialized_data = serializer_.serialize(traj_msg);

    // 3. SwarmComm 메시지에 담아 발행
    auto packet = std::make_unique<jfi_comm::msg::SwarmComm>();
    packet->tid = 2; // TID_TRAJECTORY
    packet->payload = serialized_data;
    publisher_->publish(std::move(packet));
    
    packets_sent_in_period_++;
  }

  // 수신 및 분석 로직
  void packet_callback(const jfi_comm::msg::SwarmComm::SharedPtr msg)
  {
    packets_received_in_period_++;

    // 1. MAVLink 시퀀스 번호로 패킷 손실 계산
    if (last_rx_seq_known_) {
      // seq 번호는 255 -> 0 으로 순환하므로 이를 고려하여 계산
      uint8_t expected_seq = static_cast<uint8_t>(last_rx_seq_ + 1);
      if (msg->seq != expected_seq) {
        uint8_t diff = (msg->seq > last_rx_seq_) ? (msg->seq - last_rx_seq_ - 1) : (255 - last_rx_seq_ + msg->seq);
        lost_packets_in_period_ += diff;
      }
    }
    last_rx_seq_ = msg->seq;
    last_rx_seq_known_ = true;

    // 2. 페이로드를 다시 Trajectory 메시지로 역직렬화
    try {
      auto traj_msg = serializer_.deserialize(msg->payload);

      // 3. 원본 타임스탬프를 이용해 단방향 지연시간(One-way Latency) 계산
      rclcpp::Time start_time = traj_msg.header.stamp;
      rclcpp::Duration latency = this->get_clock()->now() - start_time;
      latency_buffer_.push_back(latency.seconds() * 1000.0);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Payload deserialization failed: %s", e.what());
    }
  }

  // 리포트 로직
  void report_callback()
  {
    double avg_latency = 0.0, max_latency = 0.0;
    if (!latency_buffer_.empty()) {
      double sum = std::accumulate(latency_buffer_.begin(), latency_buffer_.end(), 0.0);
      avg_latency = sum / latency_buffer_.size();
      max_latency = *std::max_element(latency_buffer_.begin(), latency_buffer_.end());
    }

    size_t total_expected_from_peer = packets_received_in_period_ + lost_packets_in_period_;
    double loss_rate = (total_expected_from_peer > 0) ? 
                       (static_cast<double>(lost_packets_in_period_) / total_expected_from_peer * 100.0) : 0.0;

    RCLCPP_INFO(this->get_logger(), "========== BIDIRECTIONAL REPORT (5s Interval) ==========");
    RCLCPP_INFO(this->get_logger(), "TX (Sent by me)    : %zu packets", packets_sent_in_period_);
    RCLCPP_INFO(this->get_logger(), "RX (From peer)     : %zu packets received, %zu lost (%.2f%% loss)",
                packets_received_in_period_, lost_packets_in_period_, loss_rate);
    if (!latency_buffer_.empty()) {
      RCLCPP_INFO(this->get_logger(), "One-way Latency(ms): Avg: %.2f | Max: %.2f", avg_latency, max_latency);
    }
    RCLCPP_INFO(this->get_logger(), "========================================================");

    // 다음 리포트를 위해 주기별 카운터 초기화
    packets_sent_in_period_ = 0;
    packets_received_in_period_ = 0;
    lost_packets_in_period_ = 0;
    latency_buffer_.clear();
  }

  // 멤버 변수
  rclcpp::Publisher<jfi_comm::msg::SwarmComm>::SharedPtr publisher_;
  rclcpp::Subscription<jfi_comm::msg::SwarmComm>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  
  Ros2Serializer<trajectory_msgs::msg::MultiDOFJointTrajectory> serializer_;
  
  uint32_t tx_seq_; // 내가 보내는 패킷의 순번
  uint8_t last_rx_seq_; // 상대방에게 받은 마지막 seq
  bool last_rx_seq_known_;
  
  // 리포트용 변수
  size_t packets_sent_in_period_ = 0;
  size_t packets_received_in_period_ = 0;
  size_t lost_packets_in_period_ = 0;
  std::vector<double> latency_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvaluatorNode>());
  rclcpp::shutdown();
  return 0;
}