#ifndef JFI_COMM_HPP
#define JFI_COMM_HPP

#include <string>
#include <vector>
#include <mutex>
#include <functional>
#include <thread>
#include <memory>
#include <cstring>
#include <atomic>
#include <array>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <map>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/serialization.hpp>

#include "mavlink/jfi/mavlink.h"

#pragma pack(push, 1)
struct JfiFragmentHeader
{
  uint16_t transaction_id;  // ID for this sequence of fragments
  uint8_t original_tid;     // TID of the final reassembled message
  uint8_t fragment_count;   // Total number of fragments
  uint8_t fragment_seq;     // Sequence number of this fragment (0-indexed)
};
#pragma pack(pop)

struct ReassemblyBuffer
{
  uint8_t original_tid;
  uint8_t fragment_count;
  size_t total_size = 0;
  std::vector<std::vector<uint8_t>> chunks;
  std::chrono::steady_clock::time_point last_update;

  ReassemblyBuffer(uint8_t tid, uint8_t count)
    : original_tid(tid), fragment_count(count), chunks(count)
  {
      last_update = std::chrono::steady_clock::now();
  }
};

/**
 * @class JFiComm
 * @brief A library that implements serial communication using MAVLink.
 *
 * - openPort/closePort: Open and close the serial port.
 * - send: Encode topic data into a MAVLink message and send it.
 * - recvMavLoop: Read bytes from the serial port and parse them as MAVLink.
 * - serialize_message / deserialize_message: Convert ROS2 messages to/from byte arrays.
 */
class JFiComm
{
public:
  JFiComm();
  ~JFiComm();

  // Delete copy and move operations to avoid resource duplication.
  JFiComm(const JFiComm&) = delete;
  JFiComm& operator=(const JFiComm&) = delete;
  JFiComm(JFiComm&&) = delete;
  JFiComm& operator=(JFiComm&&) = delete;

  /**
   * @brief Initialize serial communication.
   *
   * @param recv_cb       Callback function to handle received messages.
   * @param port_name     Serial port name (e.g., "/dev/ttyUSB0").
   * @param baud_rate     Baud rate (e.g., 115200).
   * @param system_id     Source system ID.
   * @param component_id  Source component ID.
   * @return true if initialization is successful.
   */
  bool init(
    std::function<void(uint8_t, uint8_t, const std::vector<uint8_t>&)> recv_cb,
    const std::string & port_name, int baud_rate, uint8_t system_id = 1, uint8_t component_id = 1
  );

  /**
   * @brief Close the serial port.
   */
  void closePort();

  /**
   * @brief Encode and send a MAVLink message.
   *
   * @param tid   Message type identifier.
   * @param data  Data to be sent.
   */
  void send(const uint8_t tid, const std::vector<uint8_t>& data);

  /**
   * @brief Continuously read from the serial port and parse MAVLink messages.
   */
  void recvMavLoop();

  /**
   * @brief Serialize a ROS2 topic message.
   * Example: auto serialized_data = serialize_message(image_msg);
   *
   * @tparam T Message type.
   * @param msg Shared pointer to the ROS2 message.
   * @return Serialized message.
   */
  template <typename T>
  std::vector<uint8_t> serialize_message(const std::shared_ptr<T>& msg)
  {
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(msg.get(), &serialized_msg);

    // Convert the serialized message to a byte array.
    std::vector<uint8_t> data(
        serialized_msg.get_rcl_serialized_message().buffer,
        serialized_msg.get_rcl_serialized_message().buffer +
          serialized_msg.get_rcl_serialized_message().buffer_length
    );
    return data;
  }

  /**
   * @brief Deserialize a byte array into a ROS2 message.
   * Example: auto deserialized_msg = deserialize_message<geometry_msgs::msg::Twist>(serialized_data);
   *
   * @tparam T Message type.
   * @param data Serialized data.
   * @return Deserialized message.
   */
  template <typename T>
  T deserialize_message(const std::vector<uint8_t>& data)
  {
    T msg;
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;

    serialized_msg.reserve(data.size());
    serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

    std::memcpy(
        serialized_msg.get_rcl_serialized_message().buffer,
        data.data(),
        data.size()
    );

    serializer.deserialize_message(&serialized_msg, &msg);
    return msg;
  }

private:
  /**
   * @brief Open the serial port.
   *
   * @param port_name Serial port name.
   * @param baud_rate Baud rate.
   * @return true if the port is successfully opened.
   */
  bool openPort(const std::string & port_name, int baud_rate);

  /**
   * @brief Write data to the serial port.
   *
   * @param data Data to send.
   */
  void writeData(const std::vector<uint8_t> & data);

  /**
   * @brief Encodes and sends a single MAVLink packet.
   *
   * @param tid Message type identifier for the packet.
   * @param payload The data to be placed inside the MAVLink message payload.
   */
  void send_packet(uint8_t tid, const std::vector<uint8_t>& payload);

  /**
   * @brief Processes a received data fragment for reassembly.
   * Stores the fragment and, if all fragments have arrived, reassembles
   * the original message and triggers the user callback.
   *
   * @param src_sysid The system ID of the message sender.
   * @param raw_payload The received payload containing the fragment header and data.
   */
  void process_fragment(uint8_t src_sysid, const std::vector<uint8_t>& raw_payload);

  /**
   * @brief Cleans up incomplete reassembly buffers that have not received
   */
  void cleanup_stale_fragments();

private:
  int fd_;
  std::thread mav_recv_thread_;
  std::mutex fd_mutex_;
  std::mutex reassembly_mutex_;
  std::function<void(uint8_t, uint8_t, const std::vector<uint8_t>&)> receive_callback_;
  std::atomic<bool> running_;

  uint8_t system_id_;
  uint8_t component_id_;
  
  std::array<uint8_t, 512> rx_buffer_;

  static constexpr uint8_t FRAGMENT_TID = 255;
  std::atomic<uint16_t> next_transaction_id_{0};
  std::map<uint16_t, ReassemblyBuffer> reassembly_buffers_;
};

#endif  // JFI_COMM_HPP