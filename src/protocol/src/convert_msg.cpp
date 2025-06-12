#include "convert_msg.hpp"

acs_msgs::msg::PointNav gmcs2ros_pointnav(const GMCSProtocol& packet)
{

    if (packet.payload().size() != sizeof(StructPointNav)) {
        throw std::runtime_error("Invalid parsed_data size. Expected " +
            std::to_string(sizeof(StructPointNav)) + " != " +
            std::to_string(packet.payload().size()) + " bytes.");
    }

    StructPointNav payload;
    std::memcpy(&payload, packet.payload().data(), sizeof(StructPointNav));

    acs_msgs::msg::PointNav msg;
    msg.lon = payload.lon;
    msg.lat = payload.lat;
    msg.alt = payload.alt;
    msg.kph = payload.kph;
    msg.angle = payload.angle;

    return msg;
}

acs_msgs::msg::UVRx001 gmcs2ros_uvrx001(const GMCSProtocol& packet)
{
    
    if (packet.payload().size() != sizeof(StructUVRx001)) {
        throw std::runtime_error("Invalid parsed_data size. Expected " +
            std::to_string(sizeof(StructUVRx001)) + " != " +
            std::to_string(packet.payload().size()) + " bytes.");
    }

    StructUVRx001 payload;
    std::memcpy(&payload, packet.payload().data(), sizeof(StructUVRx001));

    acs_msgs::msg::UVRx001 msg;
    msg.op_mode = payload.op_mode;
    msg.uav_lateral_speed_kph = payload.uav_lateral_speed_kph;
    msg.uav_longitudinal_speed_kph = payload.uav_longitudinal_speed_kph;
    msg.uav_yaw_rate_dps = payload.uav_yaw_rate_dps;
    msg.uav_climb_rate_mps = payload.uav_climb_rate_mps;

    msg.ugv_steering_cmd_deg = payload.ugv_steering_cmd_deg;
    msg.usv_steering_cmd_deg = payload.usv_steering_cmd_deg;
    msg.usv_brake_cmd_pct = payload.usv_brake_cmd_pct;
    msg.ugv_brake_cmd_pct = payload.ugv_brake_cmd_pct;
    msg.usv_acc_cmd_pct = payload.usv_acc_cmd_pct;
    msg.ugv_acc_cmd_pct = payload.ugv_acc_cmd_pct;

    msg.point_nav_final_alt_m = payload.point_nav_final_alt_m;
    msg.point_nav_max_spd_kph = payload.point_nav_max_spd_kph;
    msg.point_nav_hdng_angle_deg = payload.point_nav_hdng_angle_deg;

    msg.point_nav_wgs84_lat_deg_over_10e7 = payload.point_nav_wgs84_lat_deg_over_10e7;
    msg.point_nav_wgs84_lon_deg_over_10e7 = payload.point_nav_wgs84_lon_deg_over_10e7;

    msg.point_nav_wgs84_lon_deg = payload.point_nav_wgs84_lon_deg;
    msg.point_nav_wgs84_lat_deg = payload.point_nav_wgs84_lat_deg;

    msg.point_nav_ned_north_m = payload.point_nav_ned_north_m;
    msg.point_nav_ned_east_m = payload.point_nav_ned_east_m;

    msg.takeoff_final_alt_m = payload.takeoff_final_alt_m;
    msg.takeoff_climb_rate_mps = payload.takeoff_climb_rate_mps;
    msg.takeoff_hdng_angle_deg = payload.takeoff_hdng_angle_deg;

    msg.landing_site_0_ground_1_pad = payload.landing_site_0_ground_1_pad;
    msg.landing_climb_rate_mps = payload.landing_climb_rate_mps;
    msg.landing_hdng_angle_deg = payload.landing_hdng_angle_deg;

    msg.landing_wgs84_lat_deg_over_10e7 = payload.landing_wgs84_lat_deg_over_10e7;
    msg.landing_wgs84_lon_deg_over_10e7 = payload.landing_wgs84_lon_deg_over_10e7;

    msg.landing_wgs84_lon_deg = payload.landing_wgs84_lon_deg;
    msg.landing_wgs84_lat_deg = payload.landing_wgs84_lat_deg;

    msg.landing_ned_north_m = payload.landing_ned_north_m;
    msg.landing_ned_east_m = payload.landing_ned_east_m;

    msg.uav_no_gps_satellite = payload.uav_no_gps_satellite;
    msg.uav_roll_angle_deg = payload.uav_roll_angle_deg;
    msg.uav_pitch_angle_deg = payload.uav_pitch_angle_deg;
    msg.uav_yaw_angle_deg = payload.uav_yaw_angle_deg;

    msg.uav_wgs84_pos_lat_deg_over_10e7 = payload.uav_wgs84_pos_lat_deg_over_10e7;
    msg.uav_wgs84_pos_lon_deg_over_10e7 = payload.uav_wgs84_pos_lon_deg_over_10e7;

    msg.uav_wgs84_pos_lon_deg = payload.uav_wgs84_pos_lon_deg;
    msg.uav_wgs84_pos_lat_deg = payload.uav_wgs84_pos_lat_deg;

    msg.uav_alt_m = payload.uav_alt_m;
    msg.uav_ground_speed_mps = payload.uav_ground_speed_mps;
    msg.uav_course_heading_deg = payload.uav_course_heading_deg;

    return msg;
}


// Convert a ROS 2 message directly to a GMCSProtocol serial packet (including header)
std::vector<uint8_t> ros2serial_p1(const acs_msgs::msg::OMMPointNav& msg)
{
    // TODO: should consider endianness
    StructPointNav point_data;
    point_data.lon = msg.lon;
    point_data.lat = msg.lat;
    point_data.alt = msg.alt;
    point_data.kph = msg.kph;
    point_data.angle = msg.angle;

    std::vector<uint8_t> payload(sizeof(StructPointNav));
    std::memcpy(payload.data(), &point_data, sizeof(StructPointNav));

    return GMCSProtocol::createPacket(payload);
}

std::vector<uint8_t> ros2serial_p2(const acs_msgs::msg::UVRx001& msg)
{
    StructUVRx001 data;
    data.op_mode = msg.op_mode;
    data.uav_lateral_speed_kph = msg.uav_lateral_speed_kph;
    data.uav_longitudinal_speed_kph = msg.uav_longitudinal_speed_kph;
    data.uav_yaw_rate_dps = msg.uav_yaw_rate_dps;
    data.uav_climb_rate_mps = msg.uav_climb_rate_mps;
    
    data.ugv_steering_cmd_deg = msg.ugv_steering_cmd_deg;
    data.usv_steering_cmd_deg = msg.usv_steering_cmd_deg;
    data.usv_brake_cmd_pct = msg.usv_brake_cmd_pct;
    data.ugv_brake_cmd_pct = msg.ugv_brake_cmd_pct;
    data.usv_acc_cmd_pct = msg.usv_acc_cmd_pct;
    data.ugv_acc_cmd_pct = msg.ugv_acc_cmd_pct;
    
    data.point_nav_final_alt_m = msg.point_nav_final_alt_m;
    data.point_nav_max_spd_kph = msg.point_nav_max_spd_kph;
    data.point_nav_hdng_angle_deg = msg.point_nav_hdng_angle_deg;
    
    data.point_nav_wgs84_lat_deg_over_10e7 = msg.point_nav_wgs84_lat_deg_over_10e7;
    data.point_nav_wgs84_lon_deg_over_10e7 = msg.point_nav_wgs84_lon_deg_over_10e7;
    
    data.point_nav_wgs84_lon_deg = msg.point_nav_wgs84_lon_deg;
    data.point_nav_wgs84_lat_deg = msg.point_nav_wgs84_lat_deg;
    
    data.point_nav_ned_north_m = msg.point_nav_ned_north_m;
    data.point_nav_ned_east_m = msg.point_nav_ned_east_m;
    
    data.takeoff_final_alt_m = msg.takeoff_final_alt_m;
    data.takeoff_climb_rate_mps = msg.takeoff_climb_rate_mps;
    data.takeoff_hdng_angle_deg = msg.takeoff_hdng_angle_deg;
    
    data.landing_site_0_ground_1_pad = msg.landing_site_0_ground_1_pad;
    data.landing_climb_rate_mps = msg.landing_climb_rate_mps;
    data.landing_hdng_angle_deg = msg.landing_hdng_angle_deg;
    
    data.landing_wgs84_lat_deg_over_10e7 = msg.landing_wgs84_lat_deg_over_10e7;
    data.landing_wgs84_lon_deg_over_10e7 = msg.landing_wgs84_lon_deg_over_10e7;
    
    data.landing_wgs84_lon_deg = msg.landing_wgs84_lon_deg;
    data.landing_wgs84_lat_deg = msg.landing_wgs84_lat_deg;
    
    data.landing_ned_north_m = msg.landing_ned_north_m;
    data.landing_ned_east_m = msg.landing_ned_east_m;
    
    data.uav_no_gps_satellite = msg.uav_no_gps_satellite;
    data.uav_roll_angle_deg = msg.uav_roll_angle_deg;
    data.uav_pitch_angle_deg = msg.uav_pitch_angle_deg;
    data.uav_yaw_angle_deg = msg.uav_yaw_angle_deg;
    
    data.uav_wgs84_pos_lat_deg_over_10e7 = msg.uav_wgs84_pos_lat_deg_over_10e7;
    data.uav_wgs84_pos_lon_deg_over_10e7 = msg.uav_wgs84_pos_lon_deg_over_10e7;
    
    data.uav_wgs84_pos_lon_deg = msg.uav_wgs84_pos_lon_deg;
    data.uav_wgs84_pos_lat_deg = msg.uav_wgs84_pos_lat_deg;
    
    data.uav_alt_m = msg.uav_alt_m;
    data.uav_ground_speed_mps = msg.uav_ground_speed_mps;
    data.uav_course_heading_deg = msg.uav_course_heading_deg;
    
    // Create a payload vector from the struct
    std::vector<uint8_t> payload(sizeof(StructUVRx001));
    std::memcpy(payload.data(), &data, sizeof(StructUVRx001));
    
    // Generate and return a complete GMCSProtocol packet from the payload
    return GMCSProtocol::createPacket(payload);
}