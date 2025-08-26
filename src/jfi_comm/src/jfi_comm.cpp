#include "jfi_comm.hpp"

#include <cstring>
#include <stdexcept>
#include <cerrno>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>

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

bool JFiComm::init(
  std::function<void(uint8_t, uint8_t, const std::vector<uint8_t>&)> recv_cb,
  const std::string & port_name, int baud_rate,uint8_t system_id, uint8_t component_id
)
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

void JFiComm::send(const uint8_t tid, const std::vector<uint8_t>& raw_data)
{
  const size_t max_payload_size = MAVLINK_MSG_JFI_FIELD_DATA_LEN;

  if (raw_data.size() <= max_payload_size) {
    send_packet(tid, raw_data);
  } else {
    const size_t header_size = sizeof(JfiFragmentHeader);
    const size_t max_chunk_size = max_payload_size - header_size;
    
    if (max_chunk_size <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "[send] Max payload size is too small for fragmentation header. Dropping.");
        return;
    }

    const uint8_t fragment_count = static_cast<uint8_t>(std::ceil(static_cast<double>(raw_data.size()) / max_chunk_size));
    const uint16_t transaction_id = next_transaction_id_++;

    RCLCPP_INFO(rclcpp::get_logger("JFiComm"), 
        "[send] Fragmenting tid %d (%zu bytes) into %d chunks. Transaction ID: %u", 
        tid, raw_data.size(), fragment_count, transaction_id);

    for (uint8_t i = 0; i < fragment_count; ++i) {
        JfiFragmentHeader header;
        header.transaction_id = transaction_id;
        header.original_tid = tid;
        header.fragment_count = fragment_count;
        header.fragment_seq = i;

        size_t offset = i * max_chunk_size;
        size_t chunk_size = std::min(max_chunk_size, raw_data.size() - offset);

        std::vector<uint8_t> payload;
        payload.resize(header_size + chunk_size);

        std::memcpy(payload.data(), &header, header_size);
        std::memcpy(payload.data() + header_size, raw_data.data() + offset, chunk_size);
        
        send_packet(FRAGMENT_TID, payload);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

void JFiComm::send_packet(const uint8_t tid, const std::vector<uint8_t>& payload)
{
  mavlink_message_t mavlink_msg;
  mavlink_jfi_t jfi_msg;
  jfi_msg.tid = tid;

  if (payload.size() > MAVLINK_MSG_JFI_FIELD_DATA_LEN) {
    RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), 
      "[send_packet] Payload for tid %d is too large (%zu bytes). Max is %d. Dropping.", 
      tid, payload.size(), MAVLINK_MSG_JFI_FIELD_DATA_LEN);
    return;
  }
  
  jfi_msg.len = payload.size();
  std::memset(jfi_msg.data, 0, sizeof(jfi_msg.data));
  std::memcpy(jfi_msg.data, payload.data(), jfi_msg.len);

  mavlink_msg_jfi_encode(system_id_, component_id_, &mavlink_msg, &jfi_msg);
  
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  size_t len = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);
  
  writeData(std::vector<uint8_t>(buffer, buffer + len));
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

void JFiComm::recvMavLoop()
{
  mavlink_message_t message;
  mavlink_status_t status;

  auto last_cleanup_time = std::chrono::steady_clock::now();
  
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
    
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_cleanup_time).count() > 5) {
        cleanup_stale_fragments();
        last_cleanup_time = now;
    }

    ssize_t n = ::read(local_fd, rx_buffer_.data(), rx_buffer_.size());
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_ERROR(rclcpp::get_logger("JFiComm"), "[recvMavLoop] read() failed: %s", strerror(errno));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    } else if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    
    // Process received bytes.
    for (ssize_t i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer_[i], &message, &status) == 1) {
        if (message.msgid == MAVLINK_MSG_ID_JFI) {
          mavlink_jfi_t jfi_msg;
          mavlink_msg_jfi_decode(&message, &jfi_msg);

          std::vector<uint8_t> data(jfi_msg.data, jfi_msg.data + jfi_msg.len);
          uint8_t src_sysid = message.sysid;

          if (jfi_msg.tid == FRAGMENT_TID) {
            process_fragment(src_sysid, data);
          } else {
            if (receive_callback_) {
              receive_callback_(jfi_msg.tid, src_sysid, data);
            }
          }
        }
        else {
          RCLCPP_WARN(rclcpp::get_logger("JFiComm"), "[recvMavLoop] Unknown message ID: %d", message.msgid);
        }
      }
    }
  }
}

void JFiComm::process_fragment(uint8_t src_sysid, const std::vector<uint8_t>& raw_payload)
{
    if (raw_payload.size() < sizeof(JfiFragmentHeader)) {
        RCLCPP_WARN(rclcpp::get_logger("JFiComm"), "[process_fragment] Fragment packet is too small. Dropping.");
        return;
    }

    JfiFragmentHeader header;
    std::memcpy(&header, raw_payload.data(), sizeof(JfiFragmentHeader));

    std::lock_guard<std::mutex> lock(reassembly_mutex_);

    auto it = reassembly_buffers_.find(header.transaction_id);
    if (it == reassembly_buffers_.end()) {
        // First fragment for this transaction
        RCLCPP_INFO(rclcpp::get_logger("JFiComm"),
            "[process_fragment] New transaction %u. Expecting %d fragments for tid %d.",
            header.transaction_id, header.fragment_count, header.original_tid);
        reassembly_buffers_.emplace(header.transaction_id, ReassemblyBuffer(header.original_tid, header.fragment_count));
        it = reassembly_buffers_.find(header.transaction_id);
    }
    
    auto& buffer = it->second;

    // Check for inconsistencies
    if (buffer.original_tid != header.original_tid || buffer.fragment_count != header.fragment_count) {
        RCLCPP_WARN(rclcpp::get_logger("JFiComm"), "[process_fragment] Fragment inconsistency for transaction %u. Dropping.", header.transaction_id);
        reassembly_buffers_.erase(it);
        return;
    }

    if (header.fragment_seq < buffer.fragment_count && buffer.chunks[header.fragment_seq].empty()) {
        // Store the data part of the payload
        const size_t header_size = sizeof(JfiFragmentHeader);
        buffer.chunks[header.fragment_seq].assign(
            raw_payload.begin() + header_size,
            raw_payload.end()
        );
        buffer.total_size += buffer.chunks[header.fragment_seq].size();
        buffer.last_update = std::chrono::steady_clock::now();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("JFiComm"), "[process_fragment] Duplicate or invalid fragment sequence %d for transaction %u.", header.fragment_seq, header.transaction_id);
        return; // Ignore duplicate fragments
    }

    // Check if all fragments have arrived
    size_t received_count = 0;
    for(const auto& chunk : buffer.chunks) {
        if (!chunk.empty()) {
            received_count++;
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("JFiComm"), "[process_fragment] Received fragment %zu/%d for transaction %u.", received_count, buffer.fragment_count, header.transaction_id);

    if (received_count == buffer.fragment_count) {
        RCLCPP_INFO(rclcpp::get_logger("JFiComm"), "[process_fragment] Reassembly complete for transaction %u.", header.transaction_id);
        
        std::vector<uint8_t> reassembled_data;
        reassembled_data.reserve(buffer.total_size);
        for(const auto& chunk : buffer.chunks) {
            reassembled_data.insert(reassembled_data.end(), chunk.begin(), chunk.end());
        }

        if (receive_callback_) {
            receive_callback_(buffer.original_tid, src_sysid, reassembled_data);
        }
        
        reassembly_buffers_.erase(it);
    }
}

void JFiComm::cleanup_stale_fragments()
{
    std::lock_guard<std::mutex> lock(reassembly_mutex_);
    if (reassembly_buffers_.empty()) return;

    auto now = std::chrono::steady_clock::now();
    for (auto it = reassembly_buffers_.begin(); it != reassembly_buffers_.end(); ) {
        if (std::chrono::duration_cast<std::chrono::seconds>(now - it->second.last_update).count() > 10) {
            RCLCPP_WARN(rclcpp::get_logger("JFiComm"), "[cleanup] Timing out incomplete transaction %u.", it->first);
            it = reassembly_buffers_.erase(it);
        } else {
            ++it;
        }
    }
}