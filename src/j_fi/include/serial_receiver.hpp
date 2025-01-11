#ifndef SERIAL_RECEIVER_HPP
#define SERIAL_RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
// #include <serial/serial.h>
#include <vector>
#include <chrono>

using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleStatus;
using namespace std::chrono_literals;

class SerialReceiver : public rclcpp::Node
{
public:
    SerialReceiver();
    ~SerialReceiver();

private:
    void receive_serial_data();
    void deserialize_data(const std::vector<uint8_t> &data);
    void timer_cb();
    void publish_vehicle_status();
    void publish_trajectory_setpoint();

    rclcpp::Publisher<VehicleStatus>::SharedPtr vehicle_status_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

    // serial::Serial serial_port_;

    TrajectorySetpoint::SharedPtr trajectory_data_;
    VehicleStatus::SharedPtr vehicle_status_data_;
};

#endif // SERIAL_RECEIVER_HPP