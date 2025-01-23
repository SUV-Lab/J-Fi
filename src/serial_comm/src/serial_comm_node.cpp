#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "mavlink_serial_comm.hpp"

using namespace std::chrono_literals;

/**
 * @class SerialCommNode
 * @brief 예시 ROS2 노드: MavlinkSerialComm 라이브러리를 사용하여
 *        /to_serial → 시리얼 송신,
 *        시리얼 수신 → /from_serial 로 Publish
 */
class SerialCommNode : public rclcpp::Node
{
public:
  SerialCommNode()
  : Node("serial_comm_node")
  {
    // ---------------------------
    // 1) 파라미터 선언 + 획득
    // ---------------------------
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);

    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    // ---------------------------
    // 2) 시리얼 포트 열기
    // ---------------------------
    bool ok = mavlink_comm_.openPort(port_name_, baud_rate_);
    if(!ok) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open port: %s (baud=%d)",
                   port_name_.c_str(), baud_rate_);
      // 노드 실행 불가하므로 종료
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Opened port: %s (baud=%d)",
                port_name_.c_str(), baud_rate_);

    // ---------------------------
    // 3) 콜백 등록: 수신된 MAVLink 메시지 처리
    // ---------------------------
    mavlink_comm_.setReceiveCallback(
      [this](const mavlink_message_t & msg)
      {
        this->handleMavlinkMessage(msg);
      }
    );

    // ---------------------------
    // 4) ROS Pub/Sub 생성
    // ---------------------------
    // (가) /to_serial 구독 → mavlink_comm_.send()
    sub_to_serial_ = this->create_subscription<std_msgs::msg::String>(
      "to_serial", 
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        // 토픽 데이터 → 라이브러리 send()
        mavlink_comm_.send(msg->data);
        RCLCPP_INFO(this->get_logger(), 
                    "[SUB] to_serial: '%s' -> send()", 
                    msg->data.c_str());
      }
    );

    // (나) /from_serial 퍼블리셔 → 수신된 데이터 Publish
    pub_from_serial_ = this->create_publisher<std_msgs::msg::String>(
      "from_serial", 
      10
    );

    // ---------------------------
    // 5) 타이머 생성
    // ---------------------------
    // 주기적으로 송신 버퍼 + 시리얼 수신 처리
    main_timer_ = this->create_wall_timer(
      50ms, 
      [this]() {
        // 1개씩 송신
        mavlink_comm_.checkSendBuffer();
        // 수신 (수신 완료 시 setReceiveCallback()에서 handleMavlinkMessage 호출)
        mavlink_comm_.readMavlinkMessages();
      }
    );

    RCLCPP_INFO(this->get_logger(), 
                "SerialCommNode started with port=%s, baud=%d",
                port_name_.c_str(), baud_rate_);
  }

  ~SerialCommNode()
  {
    // 노드 종료 시 시리얼 포트 닫기
    mavlink_comm_.closePort();
  }

private:
  /**
   * @brief MAVLink 메시지 수신 콜백
   *        (라이브러리에서 parse 완료 후 호출)
   */
  void handleMavlinkMessage(const mavlink_message_t & msg)
  {
    if(msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
      mavlink_statustext_t st;
      mavlink_msg_statustext_decode(&msg, &st);

      std::string text((char*)st.text);

      // 1) /from_serial로 Publish
      auto ros_msg = std_msgs::msg::String();
      ros_msg.data = text;
      pub_from_serial_->publish(ros_msg);

      RCLCPP_INFO(this->get_logger(),
                  "[RECV] MAVLINK_MSG_ID_STATUSTEXT: '%s'",
                  text.c_str());
    }
    else {
      // 다른 msgid는 로그만 출력
      RCLCPP_INFO(this->get_logger(),
                  "[RECV] msgid=%u, len=%u",
                  msg.msgid, msg.len);
    }
  }

private:
  // 라이브러리 객체
  MavlinkSerialComm mavlink_comm_;

  // 파라미터
  std::string port_name_;
  int baud_rate_;

  // ROS 구성
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_to_serial_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_from_serial_;
  rclcpp::TimerBase::SharedPtr main_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialCommNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
