#ifndef SERIAL_COMM_NODE_HPP
#define SERIAL_COMM_NODE_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

#include "serial_comm.hpp"
#include "serial_relay/msg/serial_raw_data.hpp"


using namespace std::chrono_literals;

/**
 * @class RelayNode
 * @brief ROS2 node example that uses the JFiComm library to bridge ROS2 topics and MAVLink serial communication.
 *
 *  - Subscribes to /to_serial -> sends data using the library
 *  - When a MAVLink message is received, converts it and publishes to /from_serial
 */
class RelayNode : public rclcpp::Node
{
public:

public:
    RelayNode();
    ~RelayNode();

private:
    /**
     * @brief Callback invoked when a MAVLink message is received.
     *
     * @param msg Received message data as a byte vector.
     */
    void handleMessage(const std::vector<uint8_t>& msg);

private:
    SerialComm       serial_comm_;

    // Parameters
    std::string     serial_port_;
    int             baud_rate_;
    std::string     sub_topic_name_;
    std::string     pub_topic_name_;

    // ROS subscriptions and publishers
    rclcpp::Subscription<serial_relay::msg::SerialRawData>::SharedPtr serial_raw_data_sub_;
    rclcpp::Publisher<serial_relay::msg::SerialRawData>::SharedPtr serial_raw_data_pub_;
};

#endif  // SERIAL_COMM_NODE_HPP
