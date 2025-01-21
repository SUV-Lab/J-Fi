/** @file
 *    @brief MAVLink comm protocol testsuite generated from rover.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ROVER_TESTSUITE_H
#define ROVER_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_rover(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_rover(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_trajectory_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TRAJECTORY_SETPOINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_trajectory_setpoint_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,185
    };
    mavlink_trajectory_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pos_x = packet_in.pos_x;
        packet1.pos_y = packet_in.pos_y;
        packet1.pos_z = packet_in.pos_z;
        packet1.vel_x = packet_in.vel_x;
        packet1.vel_y = packet_in.vel_y;
        packet1.vel_z = packet_in.vel_z;
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.jerk_x = packet_in.jerk_x;
        packet1.jerk_y = packet_in.jerk_y;
        packet1.jerk_z = packet_in.jerk_z;
        packet1.yaw = packet_in.yaw;
        packet1.yawspeed = packet_in.yawspeed;
        packet1.seq_debug = packet_in.seq_debug;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_setpoint_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_trajectory_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_setpoint_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.vel_x , packet1.vel_y , packet1.vel_z , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.jerk_x , packet1.jerk_y , packet1.jerk_z , packet1.yaw , packet1.yawspeed , packet1.seq_debug );
    mavlink_msg_trajectory_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.vel_x , packet1.vel_y , packet1.vel_z , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.jerk_x , packet1.jerk_y , packet1.jerk_z , packet1.yaw , packet1.yawspeed , packet1.seq_debug );
    mavlink_msg_trajectory_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_trajectory_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_setpoint_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.vel_x , packet1.vel_y , packet1.vel_z , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.jerk_x , packet1.jerk_y , packet1.jerk_z , packet1.yaw , packet1.yawspeed , packet1.seq_debug );
    mavlink_msg_trajectory_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TRAJECTORY_SETPOINT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TRAJECTORY_SETPOINT) != NULL);
#endif
}

static void mavlink_test_vehicle_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VEHICLE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_vehicle_status_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,53,120,187
    };
    mavlink_vehicle_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.armed_time = packet_in.armed_time;
        packet1.arming_state = packet_in.arming_state;
        packet1.nav_state = packet_in.nav_state;
        packet1.seq_debug = packet_in.seq_debug;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vehicle_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_vehicle_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vehicle_status_pack(system_id, component_id, &msg , packet1.timestamp , packet1.armed_time , packet1.arming_state , packet1.nav_state , packet1.seq_debug );
    mavlink_msg_vehicle_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vehicle_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.armed_time , packet1.arming_state , packet1.nav_state , packet1.seq_debug );
    mavlink_msg_vehicle_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_vehicle_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vehicle_status_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.armed_time , packet1.arming_state , packet1.nav_state , packet1.seq_debug );
    mavlink_msg_vehicle_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VEHICLE_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VEHICLE_STATUS) != NULL);
#endif
}

static void mavlink_test_rover(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_trajectory_setpoint(system_id, component_id, last_msg);
    mavlink_test_vehicle_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ROVER_TESTSUITE_H
