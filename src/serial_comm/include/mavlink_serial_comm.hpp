#ifndef MAVLINK_SERIAL_COMM_HPP
#define MAVLINK_SERIAL_COMM_HPP

#include <string>
#include <vector>
#include <mutex>
#include <functional>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "mavlink/common/mavlink.h"

/**
 * @class JFiComm
 * @brief A library that implements serial communication with MAVLink
 *
 * - openPort/closePort: Open and close the serial port
 * - send: Convert topic data into a MAVLink message, then store in the send buffer
 * - checkSendBuffer: Send data from the buffer to the serial port one by one
 * - readMavlinkMessages: Read bytes from the serial port and parse them as MAVLink
 * - When a message is fully parsed, call the callback function (set via setReceiveCallback)
 */
class MavlinkSerialComm
{
public:
  MavlinkSerialComm();
  ~MavlinkSerialComm();

  /**
   * @brief Open the serial port
   * @param port_name e.g. "/dev/ttyUSB0"
   * @param baud_rate e.g. 115200
   * @return true if successfully opened, false otherwise
   */
  bool openPort(const std::string & port_name, int baud_rate);
  
  /**
   * @brief Close the serial port
   */
  void closePort();

  /**
   * @brief Convert topic data to a MAVLink message and store it in the send buffer
   * @param topic_data For example, a string from a ROS topic
   */
  void send(const std::string & topic_data);
  
  /**
   * @brief Check the send buffer and write one message at a time to the serial port
   */
  void checkSendBuffer();
  
  /**
   * @brief Read bytes from the serial port, parse them as MAVLink
   *        Should be called periodically or by an event
   */
  void readMavlinkMessages();

  /**
   * @brief Register a receive callback
   * @param cb A function that takes 'const mavlink_message_t &' and returns void
   *           This function is called when a MAVLink message is fully parsed
   */
  void setReceiveCallback(std::function<void(const mavlink_message_t &)> cb);

private:
  /**
   * @brief Parse MAVLink byte-by-byte
   * @param byte A single byte
   */
  void parseOneByte(uint8_t byte);

  /**
   * @brief Helper function to write raw data to the serial port
   * @param data Binary data
   */
  void writeData(const std::vector<uint8_t> & data);

private:
  int fd_;
  std::mutex fd_mutex_;

  mavlink_message_t mav_msg_;
  mavlink_status_t status_;

  std::vector<std::vector<uint8_t>> send_buffer_;
  std::mutex send_buffer_mutex_;

  std::function<void(const mavlink_message_t &)> receive_callback_;
};

#endif  // MAVLINK_SERIAL_COMM_HPP
