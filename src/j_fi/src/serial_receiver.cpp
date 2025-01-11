#include "serial_receiver.hpp"
#include <cstring>

SerialReceiver::SerialReceiver() : Node("serial_receiver")
{
    const std::string topic_prefix_out = "leader/";

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(topic_prefix_out + "target_position", qos);
    vehicle_status_pub_ = this->create_publisher<VehicleStatus>(topic_prefix_out + "vehicle_status", qos);

    // Periodic timer to read serial data
    this->create_wall_timer(100ms, std::bind(&SerialReceiver::receive_serial_data, this));
    this->create_wall_timer(200ms, std::bind(&SerialReceiver::timer_cb, this));
}

SerialReceiver::~SerialReceiver()
{
    // if (serial_port_.isOpen())
    // {
    //     serial_port_.close();
    // }
}

void SerialReceiver::receive_serial_data()
{
    // if (serial_port_.available())
    // {
    //     try
    //     {
    //         std::vector<uint8_t> received_data(serial_port_.available());
    //         serial_port_.read(received_data, received_data.size());
    //         deserialize_data(received_data);
    //     }
    //     catch (const std::exception &e)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Error receiving serial data: %s", e.what());
    //     }
    // }
}

void SerialReceiver::deserialize_data(const std::vector<uint8_t> &data)
{
    try
    {
        if (data.size() >= 25)
        {
            // TODO: Desirialize_data using MAVLINK

            // trajectory_data_ init
            // vehicle_status_data_ init
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received data size is too small");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error deserializing data: %s", e.what());
    }
}

void SerialReceiver::timer_cb()
{
    publish_vehicle_status();
    publish_trajectory_setpoint();
}

void SerialReceiver::publish_vehicle_status()
{
    VehicleStatus msg{};
    msg.arming_state = vehicle_status_data_->arming_state;
    msg.nav_state = vehicle_status_data_->nav_state;

    vehicle_status_pub_->publish(msg);
}

void SerialReceiver::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    msg.timestamp = trajectory_data_->timestamp;
    msg.position[0] = trajectory_data_->position[0];
    msg.position[1] = trajectory_data_->position[1];
    msg.velocity[0] = trajectory_data_->velocity[0];

    msg.position[2] = trajectory_data_->position[2];
    msg.acceleration[0] = trajectory_data_->acceleration[0];
    msg.acceleration[1] = trajectory_data_->acceleration[1];
    msg.acceleration[2] = trajectory_data_->acceleration[2];

    msg.jerk[1] = trajectory_data_->jerk[1];
    msg.jerk[2] = trajectory_data_->jerk[2];

    trajectory_setpoint_pub_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReceiver>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
