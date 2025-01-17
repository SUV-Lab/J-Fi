#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include "mavlink/jfi/mavlink.h"

class JFiTransportNode : public rclcpp::Node {
public:
    JFiTransportNode()
        : Node("serial_comm_node"),
          running_(true) {
        // Open the serial port
        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            rclcpp::shutdown();
        }
        configure_serial_port();

        // Start threads for sending and receiving
        receive_thread_ = std::thread(&JFiTransportNode::receive_loop, this);
        send_thread_ = std::thread(&JFiTransportNode::send_loop, this);
    }

    ~JFiTransportNode() {
        // Stop the threads
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
        if (send_thread_.joinable()) {
            send_thread_.join();
        }

        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    int serial_fd_;
    std::thread receive_thread_;
    std::thread send_thread_;
    std::atomic<bool> running_; // Flag to control thread execution

    void configure_serial_port() {
        struct termios tty;

        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes");
            rclcpp::shutdown();
        }

        // Serial port settings
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes");
            rclcpp::shutdown();
        }
    }

    void receive_loop() {
        RCLCPP_INFO(this->get_logger(), "Receive thread started.");
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t byte;

        while (running_) {
            // Read bytes from the serial port
            if (read(serial_fd_, &byte, 1) > 0) {
                if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                    // Log received message
                    RCLCPP_INFO(this->get_logger(), "Received MAVLink Message: ID = %d, SEQ = %d",
                                msg.msgid, msg.seq);
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Receive thread stopped.");
    }

    void send_loop() {
        RCLCPP_INFO(this->get_logger(), "Send thread started.");
        mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        while (running_) {
            // Prepare a sample MAVLink message
            float data[63] = {0, 1, 2.0, 3.4, -5, -6.7};
            mavlink_msg_custom_message_pack(
                1,                  // System ID
                200,                // Component ID
                &msg,               // Pointer to mavlink_message_t
                42,                 // Target ID
                99,                 // Message ID
                data                // Data array
            );

            // Send the message
            int len = mavlink_msg_to_send_buffer(buffer, &msg);
            if (write(serial_fd_, buffer, len) > 0) {
                RCLCPP_INFO(this->get_logger(), "Sent MAVLink Message: ID = %d", msg.msgid);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send MAVLink message");
            }

            // Sleep for 100ms before sending the next message
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "Send thread stopped.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JFiTransportNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
