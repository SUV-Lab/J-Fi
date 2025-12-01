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
          std::vector<uint8_t> payload(jfi_msg_.data, jfi_msg_.data + jfi_msg_.len);
          if (payload.size() < 2) {
              RCLCPP_WARN(rclcpp::get_logger("JFiComm"), "Wrong payload size: %zu, expected at least 2 bytes", payload.size());
              continue;
          }

          uint8_t seq = payload[0];
          uint8_t total = payload[1];
          std::vector<uint8_t> chunk(payload.begin() + 2, payload.end());

          std::vector<uint8_t> full_compressed;
          if (total == 1) {
              full_compressed = std::move(chunk);
          } else {
              std::lock_guard<std::mutex> lock(chunk_mutex_);
              auto& buffer = chunk_buffers_[jfi_msg_.tid];
              auto now = std::chrono::steady_clock::now();

              // Check for stale chunks (timeout: 500ms)
              if (!buffer.chunks.empty()) {
                  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - buffer.last_update).count();
                  if (elapsed > 500) {
                      RCLCPP_WARN(rclcpp::get_logger("JFiComm"),
                                  "TID=%d: Chunk timeout (%ld ms). Discarding stale chunks.",
                                  jfi_msg_.tid, elapsed);
                      buffer.chunks.clear();
                      buffer.expected_total = 0;
                  }
              }

              // Check if this is a new chunked message
              if (seq == 0) {
                  if (!buffer.chunks.empty()) {
                      RCLCPP_WARN(rclcpp::get_logger("JFiComm"),
                                  "TID=%d: New chunked message started (seq=0, total=%d) while previous incomplete chunks exist. Discarding old chunks.",
                                  jfi_msg_.tid, total);
                  }
                  buffer.chunks.clear();
                  buffer.expected_total = total;
              }

              // Check if total count changed (indicates new message started mid-reception)
              if (buffer.expected_total != 0 && buffer.expected_total != total) {
                  RCLCPP_WARN(rclcpp::get_logger("JFiComm"),
                              "TID=%d: Total count mismatch (expected %d, got %d). New message started. Discarding old chunks.",
                              jfi_msg_.tid, buffer.expected_total, total);
                  buffer.chunks.clear();
                  buffer.expected_total = total;
              }

              if (buffer.chunks.size() < total) buffer.chunks.resize(total);
              buffer.chunks[seq] = std::move(chunk);
              buffer.last_update = now;
              if (buffer.expected_total == 0) buffer.expected_total = total;

              // Count received chunks
              size_t received_count = 0;
              for (const auto& c : buffer.chunks) {
                  if (!c.empty()) received_count++;
              }

              RCLCPP_DEBUG(rclcpp::get_logger("JFiComm"),
                          "TID=%d: Received chunk %d/%d (total received: %zu/%d)",
                          jfi_msg_.tid, seq, total, received_count, total);

              bool complete = (received_count == total);
              if (!complete) continue;

              for (const auto& c : buffer.chunks) {
                  full_compressed.insert(full_compressed.end(), c.begin(), c.end());
              }
              buffer.chunks.clear();
              buffer.expected_total = 0;

              RCLCPP_DEBUG(rclcpp::get_logger("JFiComm"),
                          "TID=%d: All chunks received, combined size=%zu",
                          jfi_msg_.tid, full_compressed.size());
          }

          std::string decompressed_str;
          if (!snappy::Uncompress(reinterpret_cast<const char*>(full_compressed.data()), full_compressed.size(), &decompressed_str)) {
              RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "Failed to decompress data for TID=%d, compressed_size=%zu", jfi_msg_.tid, full_compressed.size());
              continue;
          }
          std::vector<uint8_t> decompressed(decompressed_str.begin(), decompressed_str.end());

          if (receive_callback_) {
              receive_callback_(jfi_msg_.tid, decompressed);
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

void JFiComm::send(const uint8_t tid, const std::vector<uint8_t>& data) {
    std::string compressed_str;
    snappy::Compress(reinterpret_cast<const char*>(data.data()), data.size(), &compressed_str);
    std::vector<uint8_t> compressed(compressed_str.begin(), compressed_str.end());

    if (compressed.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "Failed to compress data, empty input");
        return;
    }

    // Log compression info for debugging
    RCLCPP_DEBUG(rclcpp::get_logger("JFiComm"), "Sending TID=%d: original_size=%zu, compressed_size=%zu",
                 tid, data.size(), compressed.size());

    const size_t max_chunk = 248;
    if (compressed.size() <= max_chunk) {
        std::vector<uint8_t> payload(2 + compressed.size());
        payload[0] = 0;  // seq
        payload[1] = 1;  // total
        std::copy(compressed.begin(), compressed.end(), payload.begin() + 2);
        mavlink_message_t mavlink_msg;
        mavlink_jfi_t jfi_msg;
        jfi_msg.tid = tid;
        jfi_msg.len = payload.size();
        std::memset(jfi_msg.data, 0, sizeof(jfi_msg.data));
        std::memcpy(jfi_msg.data, payload.data(), jfi_msg.len);
        mavlink_msg_jfi_encode(system_id_, component_id_, &mavlink_msg, &jfi_msg);

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        std::memset(buffer, 0, sizeof(buffer));
        size_t len = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);
        writeData(std::vector<uint8_t>(buffer, buffer + len));
    } else {
        uint8_t total_chunks = static_cast<uint8_t>((compressed.size() + max_chunk - 1) / max_chunk);

        RCLCPP_DEBUG(rclcpp::get_logger("JFiComm"),
                    "TID=%d: Sending chunked message: total_chunks=%d, compressed_size=%zu",
                    tid, total_chunks, compressed.size());

        size_t offset = 0;
        for (uint8_t seq = 0; seq < total_chunks; ++seq) {
            size_t chunk_size = std::min(max_chunk, compressed.size() - offset);
            std::vector<uint8_t> payload(2 + chunk_size);
            payload[0] = seq;
            payload[1] = total_chunks;
            std::copy(compressed.begin() + offset, compressed.begin() + offset + chunk_size, payload.begin() + 2);

            mavlink_message_t mavlink_msg;
            mavlink_jfi_t jfi_msg;
            jfi_msg.tid = tid;
            jfi_msg.len = payload.size();
            std::memset(jfi_msg.data, 0, sizeof(jfi_msg.data));
            std::memcpy(jfi_msg.data, payload.data(), jfi_msg.len);
            mavlink_msg_jfi_encode(system_id_, component_id_, &mavlink_msg, &jfi_msg);

            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            std::memset(buffer, 0, sizeof(buffer));
            size_t len = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);
            writeData(std::vector<uint8_t>(buffer, buffer + len));

            RCLCPP_DEBUG(rclcpp::get_logger("JFiComm"),
                        "TID=%d: Sent chunk %d/%d, size=%zu",
                        tid, seq, total_chunks, chunk_size);

            offset += chunk_size;
        }
    }
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
