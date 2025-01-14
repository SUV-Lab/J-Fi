#ifndef SERIAL_SENDER_HPP
#define SERIAL_SENDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include "mavlink/rover/mavlink.h"
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleStatus;

class SerialSender : public rclcpp::Node
{
public:
    SerialSender();
    ~SerialSender();

private:
    int serial_fd_;
    void configure_serial_port();
    void trajectory_callback(const TrajectorySetpoint::SharedPtr msg);
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
    void send_serial_data(mavlink_message_t &msg);

    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
};

#endif // SERIAL_SENDER_HPP
