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
 * Features:
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

  /**
   * @brief Initialize JFi Communication.
   * @param recv_cb Callback function to be called when a message is received.
   * @param port_name Serial port name (e.g. "/dev/ttyUSB0").
   * @param baud_rate Baud rate (e.g. 115200).
   * @return true if initialization is successful.
   */
  bool init(std::function<void(const int tid, const std::vector<uint8_t> &)> recv_cb,
            const std::string & port_name, int baud_rate);

  /**
   * @brief Close the serial port.
   */
  void closePort();

  /**
   * @brief Encode topic data into a MAVLink message and send it immediately.
   * @param tid Topic/message ID.
   * @param data Data to be sent.
   */
  void send(const uint8_t tid, const std::vector<uint8_t> & data);

  /**
   * @brief Read bytes from the serial port and parse them as MAVLink.
   * This function is run in a separate thread.
   */
  void recvMavLoop();

  /**
   * @brief Serialize a ROS2 topic message.
   * Example: auto serialized_data = serialize_message(image_msg);
   * @param msg Shared pointer to the message.
   * @return Byte array containing the serialized message.
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
   * @param data Serialized data.
   * @return The deserialized message.
   */
  template <typename T>
  T deserialize_message(const std::vector<uint8_t>& data)
  {
    T msg;
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;

    // Allocate memory for the serialized message.
    serialized_msg.reserve(data.size());
    serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

    // Copy data into the serialized message buffer.
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
   * @param port_name e.g. "/dev/ttyUSB0"
   * @param baud_rate e.g. 115200
   * @return true if the port is opened successfully, false otherwise.
   */
  bool openPort(const std::string & port_name, int baud_rate);

  /**
   * @brief Helper function to write raw data to the serial port.
   * @param data Binary data to be sent.
   */
  void writeData(const std::vector<uint8_t> & data);

private:
  int fd_;

  std::thread mav_recv_thread_;
  std::mutex fd_mutex_;

  std::function<void(const int tid, const std::vector<uint8_t> &)> receive_callback_;

  // Flag to control thread termination.
  std::atomic<bool> running_;
};

#endif  // JFI_COMM_HPP
