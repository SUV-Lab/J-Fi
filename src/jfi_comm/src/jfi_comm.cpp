#include "jfi_comm.hpp"

#include <cstring>
#include <stdexcept>
#include <cerrno>
#include <iostream>
#include <sys/ioctl.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <vector>
#include <memory>

JFiComm::JFiComm()
: fd_(-1)
{
  memset(&mav_msg_, 0, sizeof(mav_msg_));
  memset(&status_, 0, sizeof(status_));
}

JFiComm::~JFiComm()
{
  closePort();
}

bool JFiComm::init(std::function<void(const int tid, const std::vector<uint8_t> &)> recv_cb, const std::string & port_name, int baud_rate)
{
  bool ret = false;

  receive_callback_ = recv_cb;
  ret = openPort(port_name, baud_rate);

  mav_recv_thread_ = std::thread(&JFiComm::recvMavLoop, this);

  return ret;
}

/**
 * @brief Read bytes from the serial port and parse them as MAVLink
 */
void JFiComm::recvMavLoop()
{
  mavlink_message_t message;
  mavlink_status_t status;

  while (true) {
    if(fd_ < 0) {
      // sleep
      continue;
    }
    std::vector<uint8_t> rx(256);
    ssize_t n = ::read(fd_, rx.data(), rx.size());
    for (int i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, rx[i], &message, &status) == 1) {
        switch (message.msgid) {
          case MAVLINK_MSG_ID_JFI:
            {
            std::cout << "RECV" << std::endl;
            mavlink_jfi_t jfi_msg_;
            mavlink_msg_jfi_decode(&message, &jfi_msg_);
            std::vector<uint8_t> data(jfi_msg_.data, jfi_msg_.data + jfi_msg_.len);
            receive_callback_(jfi_msg_.tid, data);
            }
            break;
          default:
            std::cout <<"can not find msg ID" << std::endl;
            break;
        }
      }
    }
  }
}

/**
 * @brief Open the serial port, set 8N1 and baud_rate
 */
bool JFiComm::openPort(const std::string & port_name, int baud_rate)
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if(fd_ >= 0) {
    return true;
  }

  fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if(fd_ < 0) {
    std::cerr << "[ERROR] Failed to open " << port_name
              << ": " << strerror(errno) << std::endl;
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if(tcgetattr(fd_, &tty) != 0) {
    std::cerr << "[ERROR] tcgetattr: " << strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  speed_t speed = B115200;
  switch(baud_rate) {
  case 9600:
    speed = B9600;
    break;
  case 19200:
    speed = B19200;
    break;
  case 38400:
    speed = B38400;
    break;
  case 57600:
    speed = B57600;
    break;
  case 115200:
    speed = B115200;
    break;
  default:
    std::cerr << "[WARN] Unsupported baud rate: " << baud_rate
              << ", defaulting to 115200" << std::endl;
    speed = B115200;
    break;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits
  tty.c_cflag &= ~PARENB;  // no parity
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit
  tty.c_cflag |= CLOCAL | CREAD; // local line, enable read

  // Set raw mode
  cfmakeraw(&tty);

  if(tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "[ERROR] tcsetattr: " << strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  tcflush(fd_, TCIOFLUSH);

  std::cout << "[INFO] Opened port " << port_name 
            << " at " << baud_rate << " bps" << std::endl;
  return true;
}

void JFiComm::closePort()
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if(fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
    std::cout << "[INFO] Port closed" << std::endl;
  }
}

/**
 * @brief Convert topic_data into a MAVLink message (STATUSTEXT example) and push to send_buffer_
 */
void JFiComm::send(const uint8_t tid, const std::vector<uint8_t> & data)
{
  // convert byte array to mavlink_message
  mavlink_message_t mavlink_msg;
  mavlink_jfi_t jfi_msg;
  jfi_msg.tid = tid;
  memcpy(jfi_msg.data, data.data(), data.size());
  jfi_msg.len = data.size();
  mavlink_msg_jfi_encode(1, 1, &mavlink_msg, &jfi_msg);
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  size_t len = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);

  // TODO: push to send_buffer_ to control the send rate
  writeData(std::vector<uint8_t>(buffer, buffer + len));

  std::cout << "PUSH" << std::endl;
}


/**
 * @brief A helper function to write raw data (not used in the main flow as we do direct write in checkSendBuffer)
 */
void JFiComm::writeData(const std::vector<uint8_t> & data)
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if(fd_ < 0) return;

  ssize_t written = ::write(fd_, data.data(), data.size());
  if(written < 0) {
    std::cerr << "[ERROR] writeData failed: " << strerror(errno) << std::endl;
  }
}
