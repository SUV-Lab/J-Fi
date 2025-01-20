#ifndef SERIAL_COMM_NODE_HPP
#define SERIAL_COMM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <mavlink/common/mavlink.h>

class SerialCommNode : public rclcpp::Node {
public:
    SerialCommNode();
    ~SerialCommNode();

private:
    int serial_fd_;
    std::thread receive_thread_;
    std::thread send_thread_;
    std::atomic<bool> running_; // Flag to control thread execution

    void configure_serial_port();
    void receive_loop();
    void send_loop();
};

#endif // SERIAL_COMM_NODE_HPP