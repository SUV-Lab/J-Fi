#include "jfi_comm.hpp"

#include <cstring>
#include <stdexcept>
#include <cerrno>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <vector>

JFiComm::JFiComm()
: fd_(-1),
  running_(false),
  system_id_(1),
  component_id_(1),
  rx_buffer_{}
{
}

JFiComm::~JFiComm()
{
  running_ = false;
  if (mav_recv_thread_.joinable()) {
    mav_recv_thread_.join();
  }
  closePort();
}

bool JFiComm::init(std::function<void(const int tid, const std::vector<uint8_t> &)> recv_cb,
                   const std::string & port_name, int baud_rate,
                   uint8_t system_id, uint8_t component_id)
{
  receive_callback_ = recv_cb;
  system_id_ = system_id;
  component_id_ = component_id;
  
  bool ret = openPort(port_name, baud_rate);
  
  if(ret) {
    running_ = true;
    mav_recv_thread_ = std::thread(&JFiComm::recvMavLoop, this);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "[init] Failed to open port. Receiver thread not started.");
  }
  
  return ret;
}

void JFiComm::recvMavLoop()
{
  mavlink_message_t message;
  mavlink_status_t status;
  
  while (running_) {
    int local_fd;
    {
      std::lock_guard<std::mutex> lock(fd_mutex_);
      if (fd_ < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      local_fd = fd_;
    }
    
    // Use fixed buffer to avoid dynamic allocation.
    ssize_t n = ::read(local_fd, rx_buffer_.data(), rx_buffer_.size());
    if (n < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "[recvMavLoop] read() failed: %s", strerror(errno));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    } else if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    
    // Process received bytes.
    for (ssize_t i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer_[i], &message, &status) == 1) {
        if (message.msgid == MAVLINK_MSG_ID_JFI) {
          // RCLCPP_INFO(rclcpp::get_logger("JFiComm"), "RECV");
          mavlink_jfi_t jfi_msg_;
          mavlink_msg_jfi_decode(&message, &jfi_msg_);
          std::vector<uint8_t> data(jfi_msg_.data, jfi_msg_.data + jfi_msg_.len);
          if (receive_callback_) {
            receive_callback_(jfi_msg_.tid, data);
          }
        } else {
          RCLCPP_WARN(rclcpp::get_logger("JFiComm"), "[recvMavLoop] Unknown message ID: %d", message.msgid);
        }
      }
    }
  }
}

bool JFiComm::openPort(const std::string & port_name, int baud_rate)
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if (fd_ >= 0) {
    return true;
  }

  fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "[openPort] Failed to open %s: %s", port_name.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "[openPort] tcgetattr failed: %s", strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Set baud rate.
  speed_t speed = B115200;
  switch (baud_rate) {
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
      RCLCPP_WARN(rclcpp::get_logger("JFiComm"), "[openPort] Unsupported baud rate: %d, defaulting to 115200", baud_rate);
      speed = B115200;
      break;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // Configure for 8N1.
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CLOCAL | CREAD;

  // Set raw mode.
  cfmakeraw(&tty);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "[openPort] tcsetattr failed: %s", strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  tcflush(fd_, TCIOFLUSH);

  RCLCPP_INFO(rclcpp::get_logger("JFiComm"), "[openPort] Opened port %s at %d bps", port_name.c_str(), baud_rate);
  return true;
}

void JFiComm::closePort()
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("JFiComm"), "Port closed");
  }
}

void JFiComm::send(const uint8_t tid, const std::vector<uint8_t> & data)
{
  mavlink_message_t mavlink_msg;
  mavlink_jfi_t jfi_msg;
  jfi_msg.tid = tid;
  jfi_msg.len = data.size();

  std::memset(jfi_msg.data, 0, sizeof(jfi_msg.data)); // clear data buffer
  std::memcpy(jfi_msg.data, data.data(), jfi_msg.len);
  
  mavlink_msg_jfi_encode(system_id_, component_id_, &mavlink_msg, &jfi_msg);
  
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  memset(buffer, 0, sizeof(buffer));
  size_t len = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);
  
  // Send (future improvements may add rate control via a send buffer).
  writeData(std::vector<uint8_t>(buffer, buffer + len));
  
  // RCLCPP_INFO(rclcpp::get_logger("JFiComm"), "PUSH");
}

void JFiComm::writeData(const std::vector<uint8_t> & data)
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if (fd_ < 0)
    return;

  ssize_t written = ::write(fd_, data.data(), data.size());
  if (written < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "[writeData] write() failed: %s", strerror(errno));
  }
}
