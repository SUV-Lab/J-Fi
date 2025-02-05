#include "jfi_comm.hpp"

#include <cstring>
#include <stdexcept>
#include <cerrno>
#include <iostream>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <vector>

JFiComm::JFiComm()
: fd_(-1),
  running_(false)
{
}

JFiComm::~JFiComm()
{
  // Safely stop the receiver thread.
  running_ = false;
  if (mav_recv_thread_.joinable()) {
    mav_recv_thread_.join();
  }
  closePort();
}

bool JFiComm::init(std::function<void(const int tid, const std::vector<uint8_t> &)> recv_cb,
                   const std::string & port_name, int baud_rate)
{
  receive_callback_ = recv_cb;
  bool ret = openPort(port_name, baud_rate);
  
  if(ret) {
    running_ = true;
    mav_recv_thread_ = std::thread(&JFiComm::recvMavLoop, this);
  } else {
    std::cerr << "[ERROR] Failed to open port. Receiver thread not started." << std::endl;
  }
  
  return ret;
}

void JFiComm::recvMavLoop()
{
  mavlink_message_t message;
  mavlink_status_t status;
  
  while (running_) {
    {
      std::lock_guard<std::mutex> lock(fd_mutex_);
      if(fd_ < 0) {
        // If port is not open, wait briefly.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
    }
    
    std::vector<uint8_t> rx(256);
    ssize_t n = ::read(fd_, rx.data(), rx.size());
    if (n < 0) {
      std::cerr << "[ERROR] read() failed: " << strerror(errno) << std::endl;
      // On error, wait briefly before retrying.
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    } else if (n == 0) {
      // No data read: wait a short period and try again.
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    
    for (ssize_t i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, rx[i], &message, &status) == 1) {
        if (message.msgid == MAVLINK_MSG_ID_JFI) {
          std::cout << "RECV" << std::endl;
          mavlink_jfi_t jfi_msg_;
          mavlink_msg_jfi_decode(&message, &jfi_msg_);
          std::vector<uint8_t> data(jfi_msg_.data, jfi_msg_.data + jfi_msg_.len);
          if (receive_callback_) {
            receive_callback_(jfi_msg_.tid, data);
          }
        } else {
          std::cout << "Unknown message ID: " << message.msgid << std::endl;
        }
      }
    }
  }
}

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

  // Configure for 8N1.
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits per byte.
  tty.c_cflag &= ~PARENB;  // No parity.
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit.
  tty.c_cflag |= CLOCAL | CREAD; // Enable receiver, local mode.

  // Set raw mode.
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

void JFiComm::send(const uint8_t tid, const std::vector<uint8_t> & data)
{
  // Create and encode a MAVLink message.
  mavlink_message_t mavlink_msg;
  mavlink_jfi_t jfi_msg;
  jfi_msg.tid = tid;
  
  // Ensure data length does not exceed the maximum size.
  size_t copy_len = std::min(data.size(), sizeof(jfi_msg.data));
  std::memcpy(jfi_msg.data, data.data(), copy_len);
  jfi_msg.len = copy_len;
  
  mavlink_msg_jfi_encode(1, 1, &mavlink_msg, &jfi_msg);
  
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  size_t len = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);
  
  // Send immediately (future improvements may add rate control via a send buffer).
  writeData(std::vector<uint8_t>(buffer, buffer + len));
  
  std::cout << "PUSH" << std::endl;
}

void JFiComm::writeData(const std::vector<uint8_t> & data)
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if(fd_ < 0) return;

  ssize_t written = ::write(fd_, data.data(), data.size());
  if(written < 0) {
    std::cerr << "[ERROR] writeData failed: " << strerror(errno) << std::endl;
  }
}
