/** @file
 *	@brief MAVLink comm testsuite protocol generated from rover.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "rover.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(rover, TRAJECTORY_SETPOINT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::rover::msg::TRAJECTORY_SETPOINT packet_in{};
    packet_in.timestamp = 963497464;
    packet_in.pos_x = 45.0;
    packet_in.pos_y = 73.0;
    packet_in.pos_z = 101.0;
    packet_in.vel_x = 129.0;
    packet_in.vel_y = 157.0;
    packet_in.vel_z = 185.0;
    packet_in.acc_x = 213.0;
    packet_in.acc_y = 241.0;
    packet_in.acc_z = 269.0;
    packet_in.jerk_x = 297.0;
    packet_in.jerk_y = 325.0;
    packet_in.jerk_z = 353.0;
    packet_in.yaw = 381.0;
    packet_in.yawspeed = 409.0;

    mavlink::rover::msg::TRAJECTORY_SETPOINT packet1{};
    mavlink::rover::msg::TRAJECTORY_SETPOINT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.pos_x, packet2.pos_x);
    EXPECT_EQ(packet1.pos_y, packet2.pos_y);
    EXPECT_EQ(packet1.pos_z, packet2.pos_z);
    EXPECT_EQ(packet1.vel_x, packet2.vel_x);
    EXPECT_EQ(packet1.vel_y, packet2.vel_y);
    EXPECT_EQ(packet1.vel_z, packet2.vel_z);
    EXPECT_EQ(packet1.acc_x, packet2.acc_x);
    EXPECT_EQ(packet1.acc_y, packet2.acc_y);
    EXPECT_EQ(packet1.acc_z, packet2.acc_z);
    EXPECT_EQ(packet1.jerk_x, packet2.jerk_x);
    EXPECT_EQ(packet1.jerk_y, packet2.jerk_y);
    EXPECT_EQ(packet1.jerk_z, packet2.jerk_z);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(rover_interop, TRAJECTORY_SETPOINT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_trajectory_setpoint_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 353.0, 381.0, 409.0
    };

    mavlink::rover::msg::TRAJECTORY_SETPOINT packet_in{};
    packet_in.timestamp = 963497464;
    packet_in.pos_x = 45.0;
    packet_in.pos_y = 73.0;
    packet_in.pos_z = 101.0;
    packet_in.vel_x = 129.0;
    packet_in.vel_y = 157.0;
    packet_in.vel_z = 185.0;
    packet_in.acc_x = 213.0;
    packet_in.acc_y = 241.0;
    packet_in.acc_z = 269.0;
    packet_in.jerk_x = 297.0;
    packet_in.jerk_y = 325.0;
    packet_in.jerk_z = 353.0;
    packet_in.yaw = 381.0;
    packet_in.yawspeed = 409.0;

    mavlink::rover::msg::TRAJECTORY_SETPOINT packet2{};

    mavlink_msg_trajectory_setpoint_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.pos_x, packet2.pos_x);
    EXPECT_EQ(packet_in.pos_y, packet2.pos_y);
    EXPECT_EQ(packet_in.pos_z, packet2.pos_z);
    EXPECT_EQ(packet_in.vel_x, packet2.vel_x);
    EXPECT_EQ(packet_in.vel_y, packet2.vel_y);
    EXPECT_EQ(packet_in.vel_z, packet2.vel_z);
    EXPECT_EQ(packet_in.acc_x, packet2.acc_x);
    EXPECT_EQ(packet_in.acc_y, packet2.acc_y);
    EXPECT_EQ(packet_in.acc_z, packet2.acc_z);
    EXPECT_EQ(packet_in.jerk_x, packet2.jerk_x);
    EXPECT_EQ(packet_in.jerk_y, packet2.jerk_y);
    EXPECT_EQ(packet_in.jerk_z, packet2.jerk_z);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(rover, VEHICLE_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::rover::msg::VEHICLE_STATUS packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.armed_time = 93372036854776311ULL;
    packet_in.arming_state = 53;
    packet_in.nav_state = 120;

    mavlink::rover::msg::VEHICLE_STATUS packet1{};
    mavlink::rover::msg::VEHICLE_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.armed_time, packet2.armed_time);
    EXPECT_EQ(packet1.arming_state, packet2.arming_state);
    EXPECT_EQ(packet1.nav_state, packet2.nav_state);
}

#ifdef TEST_INTEROP
TEST(rover_interop, VEHICLE_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vehicle_status_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 53, 120
    };

    mavlink::rover::msg::VEHICLE_STATUS packet_in{};
    packet_in.timestamp = 93372036854775807ULL;
    packet_in.armed_time = 93372036854776311ULL;
    packet_in.arming_state = 53;
    packet_in.nav_state = 120;

    mavlink::rover::msg::VEHICLE_STATUS packet2{};

    mavlink_msg_vehicle_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.armed_time, packet2.armed_time);
    EXPECT_EQ(packet_in.arming_state, packet2.arming_state);
    EXPECT_EQ(packet_in.nav_state, packet2.nav_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
