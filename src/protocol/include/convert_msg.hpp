#ifndef CONVERT_MSG_HPP
#define CONVERT_MSG_HPP

#include "gmcs_proto.hpp"
#include <vector>
#include <cstring>
#include <stdexcept>

#include "geometry_msgs/msg/point.hpp"
#include "acs_msgs/msg/point_nav.hpp"
#include "acs_msgs/msg/omm_point_nav.hpp"
#include "acs_msgs/msg/uv_rx001.hpp"

using namespace gmcs_proto_lib;

#pragma pack(1)  // Disable padding

// struct PointData
struct StructPointNav
{
    double lon;
    double lat;
    float alt;
    float kph;
    float angle;
};

// struct UVRx001
struct StructUVRx001 {
    float uav_lateral_speed_kph;
    float uav_longitudinal_speed_kph;
    float uav_yaw_rate_dps;
    float uav_climb_rate_mps;
    
    float ugv_steering_cmd_deg;
    float usv_steering_cmd_deg;
    float usv_brake_cmd_pct;
    float ugv_brake_cmd_pct;
    float usv_acc_cmd_pct;
    float ugv_acc_cmd_pct;
    
    float point_nav_final_alt_m;
    float point_nav_max_spd_kph;
    float point_nav_hdng_angle_deg;
    
    int32_t point_nav_wgs84_lat_deg_over_10e7;
    int32_t point_nav_wgs84_lon_deg_over_10e7;
    
    double point_nav_wgs84_lon_deg;
    double point_nav_wgs84_lat_deg;
    
    float point_nav_ned_north_m;
    float point_nav_ned_east_m;
    
    float takeoff_final_alt_m;
    float takeoff_climb_rate_mps;
    float takeoff_hdng_angle_deg;
    
    float landing_climb_rate_mps;
    float landing_hdng_angle_deg;
    
    int32_t landing_wgs84_lat_deg_over_10e7;
    int32_t landing_wgs84_lon_deg_over_10e7;
    
    double landing_wgs84_lon_deg;
    double landing_wgs84_lat_deg;
    
    float landing_ned_north_m;
    float landing_ned_east_m;

    float uav_roll_angle_deg;
    float uav_pitch_angle_deg;
    float uav_yaw_angle_deg;
    
    int32_t uav_wgs84_pos_lat_deg_over_10e7;
    int32_t uav_wgs84_pos_lon_deg_over_10e7;
    
    double uav_wgs84_pos_lon_deg;
    double uav_wgs84_pos_lat_deg;
    
    float uav_alt_m;
    float uav_ground_speed_mps;
    float uav_course_heading_deg;

    uint8_t op_mode;
    uint8_t landing_site_0_ground_1_pad;
    uint8_t uav_no_gps_satellite;

};


// Convert parsed data into a ROS 2 message
acs_msgs::msg::PointNav gmcs2ros_pointnav(const GMCSProtocol& parsed_data);
acs_msgs::msg::UVRx001 gmcs2ros_uvrx001(const GMCSProtocol& parsed_data);

// Convert a ROS 2 message to a GMCSProtocol-compliant serial packet (including header)
std::vector<uint8_t> ros2serial_p1(const acs_msgs::msg::OMMPointNav& msg);
std::vector<uint8_t> ros2serial_p2(const acs_msgs::msg::UVRx001& msg);

#endif // CONVERT_MSG_HPP
