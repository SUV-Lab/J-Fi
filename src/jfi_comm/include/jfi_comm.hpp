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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/serialization.hpp>

#include "mavlink/jfi/mavlink.h"

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

private:
  int fd_;
  std::thread mav_recv_thread_;
  std::mutex fd_mutex_;
  std::function<void(uint8_t, uint8_t, const std::vector<uint8_t>&)> receive_callback_;
  std::atomic<bool> running_;

  uint8_t system_id_;
  uint8_t component_id_;

  std::array<uint8_t, 512> rx_buffer_;
};

#endif  // JFI_COMM_HPP