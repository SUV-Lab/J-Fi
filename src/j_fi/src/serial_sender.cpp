#include "serial_sender.hpp"
#include <cstring>

SerialSender::SerialSender() : Node("serial_sender")
{
    const std::string topic_prefix_out = "vehicle1/fmu/out/";

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    trajectory_sub_ = this->create_subscription<TrajectorySetpoint>(
        topic_prefix_out + "trajectory_setpoint", qos, std::bind(&SerialSender::trajectory_callback, this, std::placeholders::_1));

    vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
        topic_prefix_out + "vehicle_status", qos, std::bind(&SerialSender::vehicle_status_callback, this, std::placeholders::_1));
}

SerialSender::~SerialSender()
{
    // if (serial_port_.isOpen())
    // {
    //     serial_port_.close();
    // }
}

void SerialSender::trajectory_callback(const TrajectorySetpoint::SharedPtr msg)
{
    trajectory_data_ = msg;
    send_serial_data();
}

void SerialSender::vehicle_status_callback(const VehicleStatus::SharedPtr msg)
{
    vehicle_status_data_ = msg;
    send_serial_data();
}

std::vector<uint8_t> SerialSender::serialize_data()
{
    std::vector<uint8_t> serialized_data;

    if (trajectory_data_ && vehicle_status_data_)
    {
        try
        {
            // TODO: Sirialize_data using MAVLINK

            // trajectory_data_, vehicle_status_data_
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error serializing data: %s", e.what());
        }
    }

    return serialized_data;
}

void SerialSender::send_serial_data()
{
    auto serialized_data = serialize_data();

    if (!serialized_data.empty())
    {
        try
        {
            // serial_port_.write(serialized_data);
            RCLCPP_INFO(this->get_logger(), "Sent data over serial");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error sending serial data: %s", e.what());
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialSender>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
