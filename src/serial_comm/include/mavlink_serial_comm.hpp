#ifndef MAVLINK_SERIAL_COMM_HPP
#define MAVLINK_SERIAL_COMM_HPP

#include <string>
#include <vector>
#include <mutex>
#include <functional>

// termios 헤더 (POSIX 시리얼 제어)
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

// MAVLink 헤더 (common 등 필요한 dialect)
#include "mavlink/common/mavlink.h"

/**
 * @class MavlinkSerialComm
 * @brief termios.h 기반 시리얼 + MAVLink 통신 라이브러리
 * 
 * - openPort/closePort: termios로 직접 포트 열고 닫기
 * - send: 토픽 데이터 → MAVLink 직렬화 → 송신 버퍼
 * - checkSendBuffer: 버퍼에서 1개씩 꺼내 write(fd, ...)
 * - readMavlinkMessages: 시리얼로부터 바이트 읽어 MAVLink 파서
 * - 메시지 완성 시, setReceiveCallback()으로 등록된 콜백 호출
 */
class MavlinkSerialComm
{
public:
  MavlinkSerialComm();
  ~MavlinkSerialComm();

  bool openPort(const std::string & port_name, int baud_rate);
  void closePort();

  void send(const std::string & topic_data);
  void checkSendBuffer();
  void readMavlinkMessages();

  /**
   * @brief 콜백 함수 등록
   * @param cb 완성된 mavlink_message_t를 전달받는 콜백
   */
  void setReceiveCallback(std::function<void(const mavlink_message_t &)> cb);

private:
  /**
   * @brief 바이트 단위로 MAVLink 파싱
   * @param byte 단일 바이트
   */
  void parseOneByte(uint8_t byte);

  /**
   * @brief 실제 시리얼 write
   * @param data 바이너리 데이터
   */
  void writeData(const std::vector<uint8_t> & data);

  // --------------------------------------------------
  // 멤버 변수
  // --------------------------------------------------
private:
  int fd_;  ///< termios에서 사용하는 파일 디스크립터. -1이면 닫힌 상태
  std::mutex fd_mutex_;  ///< fd_ 접근 보호

  // MAVLink 파서 상태
  mavlink_message_t mav_msg_;
  mavlink_status_t status_;

  // 송신 버퍼 (MAVLink 직렬화된 바이너리)
  std::vector<std::vector<uint8_t>> send_buffer_;
  std::mutex send_buffer_mutex_;

  // 수신 콜백
  std::function<void(const mavlink_message_t &)> receive_callback_;
};

#endif  // MAVLINK_SERIAL_COMM_HPP
