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
 * @brief Open the serial port, set 8N1 and baud_rate
 */
bool MavlinkSerialComm::openPort(const std::string & port_name, int baud_rate)
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
 * @brief Convert topic_data into a MAVLink message (STATUSTEXT example) and push to send_buffer_
 */
void MavlinkSerialComm::send(const std::string & topic_data)
{
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
 * @brief Send one message at a time from send_buffer_ to the serial port
 */
void MavlinkSerialComm::checkSendBuffer()
{
  std::lock_guard<std::mutex> fd_lock(fd_mutex_);
  if(fd_ < 0) {
    return;
  }

  std::lock_guard<std::mutex> buf_lock(send_buffer_mutex_);
  if(!send_buffer_.empty()) {
    auto & front_data = send_buffer_.front();
    ssize_t written = ::write(fd_, front_data.data(), front_data.size());

    if(written < 0) {
      std::cerr << "[ERROR] write failed: " << strerror(errno) << std::endl;
    }

    send_buffer_.erase(send_buffer_.begin());
  }
}

/**
 * @brief Read bytes from the serial port and parse them as MAVLink
 */
void MavlinkSerialComm::readMavlinkMessages()
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if(fd_ < 0) {
    return;
  }

  int bytes_available = 0;
  if(ioctl(fd_, FIONREAD, &bytes_available) == -1) {
    // If an error occurs, treat as 0
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
 * @brief Parse MAVLink byte by byte. If a message is complete, call receive_callback_
 */
void MavlinkSerialComm::parseOneByte(uint8_t byte)
{
  if(mavlink_parse_char(MAVLINK_COMM_0, byte, &mav_msg_, &status_)) {
    // A message is fully parsed
    if(receive_callback_) {
      receive_callback_(mav_msg_);
    }
  }
}

/**
 * @brief Register a callback for receiving MAVLink messages
 */
void MavlinkSerialComm::setReceiveCallback(std::function<void(const mavlink_message_t &)> cb)
{
  receive_callback_ = cb;
}

/**
 * @brief A helper function to write raw data (not used in the main flow as we do direct write in checkSendBuffer)
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
