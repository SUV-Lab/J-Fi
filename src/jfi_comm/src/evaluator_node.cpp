#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <map>
#include <numeric>

using namespace std::chrono_literals;

class EvaluatorNode : public rclcpp::Node
{
public:
  EvaluatorNode() : Node("evaluator_node"), seq_(0)
  {
    // node1으로 trajectory 메시지를 보내는 퍼블리셔
    publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
        "/node1/send_trajectory", 10);

    // node2로부터 되돌아온 메시지를 받는 서브스크라이버
    subscription_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
        "/node2/received_trajectory", 10,
        std::bind(&EvaluatorNode::topic_callback, this, std::placeholders::_1));

    // 200ms마다 (초당 5회) 메시지를 보내는 타이머
    send_timer_ = this->create_wall_timer(
        200ms, std::bind(&EvaluatorNode::send_timer_callback, this));

    // 5초마다 통계 결과를 출력하는 타이머
    report_timer_ = this->create_wall_timer(
        5s, std::bind(&EvaluatorNode::report_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Evaluator node started. Sending trajectory messages every 200ms.");
  }

private:
  void send_timer_callback()
  {
    auto message = trajectory_msgs::msg::MultiDOFJointTrajectory();
    message.header.stamp = this->get_clock()->now();
    // frame_id에 순번을 문자열로 저장하여 보냅니다.
    message.header.frame_id = std::to_string(seq_);
    message.joint_names = {"joint1", "joint2", "joint3"};
    
    // 테스트를 위한 대용량 데이터 생성
    for (int i = 0; i < 2; ++i) {
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        point.transforms.resize(1);
        point.transforms[0].translation.x = 1.0 + i;
        point.velocities.resize(1);
        point.velocities[0].linear.x = 0.1 * i;
        point.accelerations.resize(1);
        point.accelerations[0].linear.x = 0.01 * i;
        message.points.push_back(point);
    }
    
    publisher_->publish(message);
    
    // 보낸 시간과 순번을 기록
    sent_times_[seq_] = message.header.stamp;
    seq_++;
  }

  void topic_callback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
  {
    uint32_t received_seq = std::stoul(msg->header.frame_id);

    auto it = sent_times_.find(received_seq);
    if (it != sent_times_.end()) {
      rclcpp::Time start_time = it->second;
      rclcpp::Duration rtt = this->get_clock()->now() - start_time;
      rtt_buffer_.push_back(rtt.seconds() * 1000.0); // ms 단위로 저장
      sent_times_.erase(it); // 처리된 항목은 맵에서 제거
    } else {
      RCLCPP_WARN(this->get_logger(), "Received a message with an unknown or duplicate sequence: %u", received_seq);
    }
  }

  void report_timer_callback()
  {
    if (rtt_buffer_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Report interval: No messages received.");
      return;
    }

    double sum = std::accumulate(rtt_buffer_.begin(), rtt_buffer_.end(), 0.0);
    double mean_rtt = sum / rtt_buffer_.size();
    double max_rtt = *std::max_element(rtt_buffer_.begin(), rtt_buffer_.end());
    
    // sent_times_ 맵에 남아있는 것은 타임아웃되어 응답받지 못한 패킷들입니다.
    size_t lost_packets = sent_times_.size();
    size_t total_sent_in_period = rtt_buffer_.size() + lost_packets;
    double loss_rate = (total_sent_in_period > 0) ? (static_cast<double>(lost_packets) / total_sent_in_period * 100.0) : 0.0;

    RCLCPP_INFO(this->get_logger(), "========== COMMS QUALITY REPORT ==========");
    RCLCPP_INFO(this->get_logger(), "Interval: 5s | Packets Sent: %zu | Received: %zu", total_sent_in_period, rtt_buffer_.size());
    RCLCPP_INFO(this->get_logger(), "Packet Loss: %.2f %%", loss_rate);
    RCLCPP_INFO(this->get_logger(), "RTT (ms)   - Avg: %.2f | Max: %.2f", mean_rtt, max_rtt);
    RCLCPP_INFO(this->get_logger(), "==========================================");

    // 다음 리포트를 위해 버퍼와 타임아웃된 패킷 기록 초기화
    rtt_buffer_.clear();
    sent_times_.clear();
  }

  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  
  uint32_t seq_;
  std::map<uint32_t, rclcpp::Time> sent_times_;
  std::vector<double> rtt_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvaluatorNode>());
  rclcpp::shutdown();
  return 0;
}