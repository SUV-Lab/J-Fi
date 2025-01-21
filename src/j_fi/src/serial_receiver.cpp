#include "serial_receiver.hpp"

SerialReceiver::SerialReceiver() : Node("serial_receiver")
{
    int rover_id;
    this->declare_parameter<int>("rover_id", 1);
    this->get_parameter("rover_id", rover_id);
    std::string sid = std::to_string(rover_id);

    int leader_id;
    this->declare_parameter<int>("leader_id", 1);
    this->get_parameter("leader_id", leader_id);
    std::string leader_sid = std::to_string(leader_id);

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

    const std::string leader_to_follower_prefix = "vehicle" + leader_sid + "/to/" + "vehicle" + sid + "/";

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(leader_to_follower_prefix + "target_position", qos);
    vehicle_status_pub_ = this->create_publisher<VehicleStatus>(leader_to_follower_prefix + "target_status", qos);

    // Periodic timer to read serial data
    timer_ = this->create_wall_timer(100ms, std::bind(&SerialReceiver::receive_serial_data, this));
}

SerialReceiver::~SerialReceiver()
{
    if (serial_fd_ >= 0)
        close(serial_fd_);
}

void SerialReceiver::configure_serial_port()
{
    struct termios tty;

    if (tcgetattr(serial_fd_, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes");
        rclcpp::shutdown();
    }

    // J.Fi Setting
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

void SerialReceiver::receive_serial_data()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;

    // Read bytes from the serial port
    while (read(serial_fd_, &byte, 1) > 0)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
        {
            // Handle received MAVLink custom message
            if (msg.msgid == MAVLINK_MSG_ID_TRAJECTORY_SETPOINT)
            {
                mavlink_trajectory_setpoint_t traj_msg;
                mavlink_msg_trajectory_setpoint_decode(&msg, &traj_msg);

                TrajectorySetpoint msg{};
                msg.timestamp = traj_msg.timestamp;
                msg.position[0] = traj_msg.pos_x;
                msg.position[1] = traj_msg.pos_y;
                msg.position[2] = traj_msg.pos_z;
                msg.velocity[0] = traj_msg.vel_x;
                msg.velocity[1] = traj_msg.vel_y;
                msg.velocity[2] = traj_msg.vel_z;

                msg.acceleration[0] = traj_msg.acc_x;
                msg.acceleration[1] = traj_msg.acc_y;
                msg.acceleration[2] = traj_msg.acc_z;
                msg.jerk[0] = traj_msg.jerk_x;
                msg.jerk[1] = traj_msg.jerk_y;
                msg.jerk[2] = traj_msg.jerk_z;
                msg.yaw = traj_msg.yaw;
                msg.yawspeed = traj_msg.seq_debug;

                trajectory_setpoint_pub_->publish(msg);
            }
            else if (msg.msgid == MAVLINK_MSG_ID_VEHICLE_STATUS)
            {
                mavlink_vehicle_status_t status_msg;
                mavlink_msg_vehicle_status_decode(&msg, &status_msg);

                VehicleStatus msg{};
                msg.timestamp = status_msg.timestamp;
                msg.armed_time = status_msg.armed_time;
                msg.arming_state = status_msg.arming_state;
                msg.nav_state = status_msg.nav_state;
                msg.latest_arming_reason = status_msg.seq_debug;

                vehicle_status_pub_->publish(msg);
            }
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReceiver>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
