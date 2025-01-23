#include "mavlink_serial_comm.hpp"

#include <cstring>
#include <stdexcept>
#include <cerrno>
#include <iostream>
#include <sys/ioctl.h>

MavlinkSerialComm::MavlinkSerialComm()
: fd_(-1)
{
  memset(&mav_msg_, 0, sizeof(mav_msg_));
  memset(&status_, 0, sizeof(status_));
}

MavlinkSerialComm::~MavlinkSerialComm()
{
  closePort();
}

/**
 * @brief termios로 시리얼 포트를 열고 8N1, baud_rate 설정
 */
bool MavlinkSerialComm::openPort(const std::string & port_name, int baud_rate)
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if(fd_ >= 0) {
    // 이미 열려 있다면 닫고 다시 여는 로직을 넣어도 됨
    return true;
  }

  // O_RDWR: 읽기/쓰기, O_NOCTTY: 이 장치를 제어 터미널로 사용하지 않음
  fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if(fd_ < 0) {
    std::cerr << "[ERROR] Failed to open " << port_name 
              << ": " << strerror(errno) << std::endl;
    return false;
  }

  // termios 설정
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if(tcgetattr(fd_, &tty) != 0) {
    std::cerr << "[ERROR] tcgetattr: " << strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 속도 설정(baud_rate -> Bxxxx 상수 변환 필요. 예: 115200 -> B115200)
  // 간단히 예시로 115200만 처리:
  speed_t speed = B115200;
  if(baud_rate == 9600) {
    speed = B9600;
  } else if(baud_rate == 19200) {
    speed = B19200;
  } else if(baud_rate == 38400) {
    speed = B38400;
  } else if(baud_rate == 57600) {
    speed = B57600;
  } else if(baud_rate == 115200) {
    speed = B115200;
  } else {
    // 실제로 더 많은 baud rate 매핑이 필요할 수 있음
    std::cerr << "[WARN] Unsupported baud rate: " << baud_rate 
              << ", defaulting to 115200" << std::endl;
    speed = B115200;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits
  tty.c_cflag &= ~PARENB;  // no parity
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit
  tty.c_cflag |= CLOCAL | CREAD; // local line, enable read

  // raw 모드
  cfmakeraw(&tty);

  // 변경 적용
  if(tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "[ERROR] tcsetattr: " << strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 버퍼 플러시
  tcflush(fd_, TCIOFLUSH);

  std::cout << "[INFO] Opened port " << port_name 
            << " at " << baud_rate << " bps (8N1)" << std::endl;
  return true;
}

void MavlinkSerialComm::closePort()
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if(fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
    std::cout << "[INFO] Port closed" << std::endl;
  }
}

/**
 * @brief 토픽 데이터 -> MAVLink 메시지 -> 직렬화 -> send_buffer_
 */
void MavlinkSerialComm::send(const std::string & topic_data)
{
  // 예) STATUSTEXT 예시
  mavlink_message_t msg;
  mavlink_statustext_t st;
  memset(&st, 0, sizeof(st));
  strncpy((char*)st.text, topic_data.c_str(), sizeof(st.text) - 1);
  st.severity = MAV_SEVERITY_INFO;

  mavlink_msg_statustext_encode(1, 200, &msg, &st);

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  size_t len = mavlink_msg_to_send_buffer(buffer, &msg);

  {
    std::lock_guard<std::mutex> lock(send_buffer_mutex_);
    send_buffer_.push_back(std::vector<uint8_t>(buffer, buffer + len));
  }
}

/**
 * @brief 송신 버퍼에서 1개씩 꺼내 write(fd, ...)
 */
void MavlinkSerialComm::checkSendBuffer()
{
  // fd 접근
  std::lock_guard<std::mutex> fd_lock(fd_mutex_);
  if(fd_ < 0) {
    return;
  }

  // 버퍼 접근
  std::lock_guard<std::mutex> buf_lock(send_buffer_mutex_);
  if(!send_buffer_.empty()) {
    auto & front_data = send_buffer_.front();
    ssize_t written = ::write(fd_, front_data.data(), front_data.size());
    if(written < 0) {
      std::cerr << "[ERROR] write failed: " << strerror(errno) << std::endl;
    } else {
      // 성공
      // std::cout << "[SEND] " << written << " bytes" << std::endl;
    }
    send_buffer_.erase(send_buffer_.begin());
  }
}

/**
 * @brief 시리얼에서 바이트를 읽고, MAVLink 파서
 */
void MavlinkSerialComm::readMavlinkMessages()
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if(fd_ < 0) {
    return;
  }

  // 얼마나 available 한지 확인하는 방법 중 하나: FIONREAD ioctl
  int bytes_available = 0;
  if(ioctl(fd_, FIONREAD, &bytes_available) == -1) {
    // 에러지만, 간단히 무시
    bytes_available = 0;
  }

  if(bytes_available > 0) {
    std::vector<uint8_t> rx(bytes_available);
    ssize_t n = ::read(fd_, rx.data(), rx.size());
    if(n > 0) {
      for(ssize_t i=0; i<n; i++){
        parseOneByte(rx[i]);
      }
    }
  }
}

/**
 * @brief byte-by-byte로 MAVLink 파싱
 */
void MavlinkSerialComm::parseOneByte(uint8_t byte)
{
  if(mavlink_parse_char(MAVLINK_COMM_0, byte, &mav_msg_, &status_)) {
    // 메시지 완성
    if(receive_callback_) {
      receive_callback_(mav_msg_);
    }
  }
}

/**
 * @brief 수신 콜백 등록
 */
void MavlinkSerialComm::setReceiveCallback(std::function<void(const mavlink_message_t &)> cb)
{
  receive_callback_ = cb;
}

/**
 * @brief 내부 write 함수 (여기서는 checkSendBuffer() 내부에서 직접 write하므로 미사용 가능)
 */
void MavlinkSerialComm::writeData(const std::vector<uint8_t> & data)
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if(fd_ < 0) return;

  ssize_t written = ::write(fd_, data.data(), data.size());
  if(written < 0) {
    std::cerr << "[ERROR] writeData failed: " << strerror(errno) << std::endl;
  }
}
