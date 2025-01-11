#ifndef SERIAL_SENDER_HPP
#define SERIAL_SENDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
// #include <serial/serial.h>
#include <vector>
#include <chrono>

using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleStatus;
using namespace std::chrono_literals;

class SerialSender : public rclcpp::Node
{
public:
    SerialSender();
    ~SerialSender();

private:
    void trajectory_callback(const TrajectorySetpoint::SharedPtr msg);
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
    std::vector<uint8_t> serialize_data();
    void send_serial_data();

    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;

    // serial::Serial serial_port_;

    TrajectorySetpoint::SharedPtr trajectory_data_;
    VehicleStatus::SharedPtr vehicle_status_data_;
};

#endif // SERIAL_SENDER_HPP