#ifndef JFI_COMM_HPP
#define JFI_COMM_HPP

#include <string>
#include <vector>
#include <mutex>
#include <functional>
#include <thread>
#include <memory>
#include <cstring>

#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <rclcpp/serialized_message.hpp>
#include <rclcpp/serialization.hpp>

#include "mavlink/jfi/mavlink.h"

/**
 * @class JFiComm
 * @brief A library that implements serial communication with MAVLink
 *
 * - openPort/closePort: Open and close the serial port
 * - send: Convert topic data into a MAVLink message, then store in the send buffer
 * - recvMavLoop: Read bytes from the serial port and parse them as MAVLink
 * - When a message is fully parsed, call the callback function (set via setReceiveCallback)
 */
class JFiComm
{
public:
  JFiComm();
  ~JFiComm();

  /**
   * @brief init JFi Communication
   */
  bool init(std::function<void(const int tid, const std::vector<uint8_t> &)> recv_cb, const std::string & port_name, int baud_rate);

  /**
   * @brief Close the serial port
   */
  void closePort();

  /**
   * @brief Convert topic data to a MAVLink message and store it in the send buffer
   * @param topic_data For example, a string from a ROS topic
   */
  void send(const uint8_t tid, const std::vector<uint8_t> & data);

  /**
   * @brief Read bytes from the serial port, parse them as MAVLink
   *        Should be called periodically or by an event
   */
  void recvMavLoop();

  /**
   * @brief serialize topic message
   *        ex) auto serialized_data = serialize_message(image_msg);
   * @param topic message
   */
  template <typename T>
  std::vector<uint8_t> serialize_message(const std::shared_ptr<T>& msg)
  {
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(msg.get(), &serialized_msg);

    // convert topic to byte array
    std::vector<uint8_t> data(
        serialized_msg.get_rcl_serialized_message().buffer,
        serialized_msg.get_rcl_serialized_message().buffer + serialized_msg.get_rcl_serialized_message().buffer_length
    );

    return data;
  }

  /**
   * @brief deserialize data
   *        ex) auto deserialized_msg = deserialize_message<geometry_msgs::msg::Twist>(serialized_data);
   * @param serialized data
   */
  template <typename T>
  T deserialize_message(const std::vector<uint8_t>& data)
  {
    T msg;
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;

    // Allocate memory for the serialized message
    serialized_msg.reserve(data.size());
    serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

    // Copy the data into the serialized message buffer
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
   * @brief Open the serial port
   * @param port_name e.g. "/dev/ttyUSB0"
   * @param baud_rate e.g. 115200
   * @return true if successfully opened, false otherwise
   */
  bool openPort(const std::string & port_name, int baud_rate);

  /**
   * @brief Helper function to write raw data to the serial port
   * @param data Binary data
   */
  void writeData(const std::vector<uint8_t> & data);

private:
  int fd_;

  std::thread mav_recv_thread_;

  std::mutex fd_mutex_;

  mavlink_message_t mav_msg_;
  mavlink_status_t status_;

  std::vector<std::vector<uint8_t>> send_buffer_;
  std::mutex send_buffer_mutex_;

  std::function<void(const int tid, const std::vector<uint8_t> &)> receive_callback_;
};

#endif  // JFI_COMM_HPP
