#include "serial_comm_node.hpp"
#include "rclcpp/rclcpp.hpp"


SerialCommNode::SerialCommNode()
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
    receive_thread_ = std::thread(&SerialCommNode::receive_loop, this);
    send_thread_ = std::thread(&SerialCommNode::send_loop, this);
}

SerialCommNode::~SerialCommNode() {
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

void SerialCommNode::configure_serial_port() {
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

void SerialCommNode::receive_loop() {
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

void SerialCommNode::send_loop() {
    RCLCPP_INFO(this->get_logger(), "Send thread started.");
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    while (running_) {
        // Create a heartbeat message
        mavlink_msg_heartbeat_pack(
            1,                  // System ID
            200,                // Component ID
            &msg,               // Pointer to mavlink_message_t
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_PX4,
            MAV_MODE_GUIDED_ARMED,
            0,
            MAV_STATE_ACTIVE
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}