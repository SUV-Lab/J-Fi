#ifndef SERIAL_RECEIVER_HPP
#define SERIAL_RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include "mavlink/rover/mavlink.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <chrono>
#include <cstring>

using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleStatus;
using namespace std::chrono_literals;

class SerialReceiver : public rclcpp::Node
{
public:
    SerialReceiver();
    ~SerialReceiver();

private:
    int serial_fd_;
    rclcpp::TimerBase::SharedPtr timer_;

    void configure_serial_port();
    void receive_serial_data();

    rclcpp::Publisher<VehicleStatus>::SharedPtr vehicle_status_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

    TrajectorySetpoint::SharedPtr trajectory_data_;
    VehicleStatus::SharedPtr vehicle_status_data_;
};

#endif // SERIAL_RECEIVER_HPP