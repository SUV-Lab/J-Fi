#ifndef JFI_TRANSCEIVER_HPP
#define JFI_TRANSCEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <atomic>
#include <mavlink/common/mavlink.h>

class JFiTransceiverNode : public rclcpp::Node
{
public:
    JFiTransceiverNode();
    ~JFiTransceiverNode();

private:
    int serial_fd_;
    std::thread receive_thread_;
    std::thread send_thread_;
    std::atomic<bool> running_; // Flag to control thread execution

    void configure_serial_port();
    void receive_loop();
    void send_loop();
};

#endif // JFI_TRANSCEIVER_HPP