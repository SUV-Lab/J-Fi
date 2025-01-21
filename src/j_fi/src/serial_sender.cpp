#include "serial_sender.hpp"
#include <cstring>

SerialSender::SerialSender() : Node("serial_sender")
{
    int rover_id;
    this->declare_parameter<int>("rover_id", 1);
    this->get_parameter("rover_id", rover_id);

    std::string sid = std::to_string(rover_id);

    std::string port_name;
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->get_parameter("port", port_name);

    serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_name.c_str());
        rclcpp::shutdown();
    }
    configure_serial_port();

    const std::string topic_prefix_out = "vehicle" + sid + "/fmu/out/";

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    trajectory_sub_ = this->create_subscription<TrajectorySetpoint>(
        topic_prefix_out + "trajectory_setpoint", qos, std::bind(&SerialSender::trajectory_callback, this, std::placeholders::_1));

    vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
        topic_prefix_out + "vehicle_status", qos, std::bind(&SerialSender::vehicle_status_callback, this, std::placeholders::_1));
}

SerialSender::~SerialSender()
{
    if (serial_fd_ >= 0)
        close(serial_fd_);
}

void SerialSender::configure_serial_port()
{
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes");
        rclcpp::shutdown();
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes");
        rclcpp::shutdown();
    }
}

void SerialSender::vehicle_status_callback(const VehicleStatus::SharedPtr dds_msg)
{
    mavlink_message_t mav_msg;

    VehicleStatus vehicle_status{};
    vehicle_status = *dds_msg;

    mavlink_msg_vehicle_status_pack(
        1,   // System ID
        200, // Component ID
        &mav_msg,
        vehicle_status.timestamp,
        vehicle_status.armed_time,
        vehicle_status.arming_state,
        vehicle_status.nav_state,
        mav_msg.seq);

    send_serial_data(mav_msg);
}

void SerialSender::trajectory_callback(const TrajectorySetpoint::SharedPtr dds_msg)
{
    mavlink_message_t mav_msg;

    TrajectorySetpoint trajectory_setpoint{};
    trajectory_setpoint = *dds_msg;

    mavlink_msg_trajectory_setpoint_pack(
        1,   // System ID
        200, // Component ID
        &mav_msg,
        trajectory_setpoint.timestamp,
        trajectory_setpoint.position[0],
        trajectory_setpoint.position[1],
        trajectory_setpoint.position[2],
        trajectory_setpoint.velocity[0],
        trajectory_setpoint.velocity[1],
        trajectory_setpoint.velocity[2],
        trajectory_setpoint.acceleration[0],
        trajectory_setpoint.acceleration[1],
        trajectory_setpoint.acceleration[2],
        trajectory_setpoint.jerk[0],
        trajectory_setpoint.jerk[1],
        trajectory_setpoint.jerk[2],
        trajectory_setpoint.yaw,
        trajectory_setpoint.yawspeed,
        mav_msg.seq);

    send_serial_data(mav_msg);
}

void SerialSender::send_serial_data(mavlink_message_t &msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    write(serial_fd_, buffer, len);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialSender>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
