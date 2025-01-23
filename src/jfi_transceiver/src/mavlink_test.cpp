#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <mavlink/common/mavlink.h>

class MavlinkTestNode : public rclcpp::Node
{
public:
    MavlinkTestNode();
    ~MavlinkTestNode();

private:
    int serial_fd_;
    std::thread send_thread_;
    std::atomic<bool> running_; // Flag to control thread execution

    void send_loop();
};

MavlinkTestNode::MavlinkTestNode() : Node("mavlink_test_node"),  running_(true)
{
    // Start threads for sending and receiving
    send_thread_ = std::thread(&MavlinkTestNode::send_loop, this);
}

MavlinkTestNode::~MavlinkTestNode() {
    // Stop the threads
    running_ = false;
    if (send_thread_.joinable()) {
        send_thread_.join();
    }
}

void MavlinkTestNode::send_loop() {
    RCLCPP_INFO(this->get_logger(), "Send thread started.");
    mavlink_message_t msg1;
    mavlink_message_t msg2;
    mavlink_message_t msg3;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    float data[63] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0,
                    00, 12, 34, 56, 78, 90,
                    000, 123, 456, 789, 000,
                    0000, 1234, 5678, 9000};
    float data2[63] = {};
    float data3[63] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1};


    while (running_) {
        // Create a test message
        mavlink_msg_test_pack(
            1,                  // System ID
            200,                // Component ID
            &msg1,               // Pointer to mavlink_message_t
            0,
            data
        );

        mavlink_msg_test_pack(
            1,                  // System ID
            200,                // Component ID
            &msg2,               // Pointer to mavlink_message_t
            0,
            data2
        );

        mavlink_msg_test_pack(
            1,                  // System ID
            200,                // Component ID
            &msg3,               // Pointer to mavlink_message_t
            0,
            data3
        );

        // print message
        int len = mavlink_msg_to_send_buffer(buffer, &msg1);
        std::cout << "MAVLink Message Info" << std::endl;
        std::cout << "Packet Length: " << len << std::endl;
        std::cout << "Payload Length: " << static_cast<int>(msg1.len) << std::endl;
        std::cout << "Payload Data (bytes): ";
        for (int i = 0; i < static_cast<int>(msg1.len); i++) {
            std::cout << static_cast<int>(msg1.payload64[i]) << " ";
        }
        std::cout << std::endl;

        int len2 = mavlink_msg_to_send_buffer(buffer, &msg2);
        std::cout << "MAVLink Message Info" << std::endl;
        std::cout << "Packet Length: " << len2 << std::endl;
        std::cout << "Payload Length: " << static_cast<int>(msg2.len) << std::endl;
        std::cout << "Payload Data (bytes): ";
        for (int i = 0; i < static_cast<int>(msg2.len); i++) {
            std::cout << static_cast<int>(msg2.payload64[i]) << " ";
        }
        std::cout << std::endl;

        int len3 = mavlink_msg_to_send_buffer(buffer, &msg3);
        std::cout << "MAVLink Message Info" << std::endl;
        std::cout << "Packet Length: " << len3 << std::endl;
        std::cout << "Payload Length: " << static_cast<int>(msg3.len) << std::endl;
        std::cout << "Payload Data (bytes): ";
        for (int i = 0; i < static_cast<int>(msg3.len); i++) {
            std::cout << static_cast<int>(msg3.payload64[i]) << " ";
        }
        std::cout << std::endl;
        // Sleep for 100ms before sending the next message
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    }
    RCLCPP_INFO(this->get_logger(), "Send thread stopped.");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavlinkTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}