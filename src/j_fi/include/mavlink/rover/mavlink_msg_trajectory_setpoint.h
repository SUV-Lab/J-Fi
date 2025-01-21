#pragma once
// MESSAGE TRAJECTORY_SETPOINT PACKING

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT 190


typedef struct __mavlink_trajectory_setpoint_t {
 uint32_t timestamp; /*<  Time*/
 float pos_x; /*<  target position x*/
 float pos_y; /*<  target position y*/
 float pos_z; /*<  target position z*/
 float vel_x; /*<  target velocity x*/
 float vel_y; /*<  target velocity y*/
 float vel_z; /*<  target velocity z*/
 float acc_x; /*<  target acceleration x*/
 float acc_y; /*<  target acceleration y*/
 float acc_z; /*<  target acceleration z*/
 float jerk_x; /*<  target jerk x*/
 float jerk_y; /*<  target jerk y*/
 float jerk_z; /*<  target jerk z*/
 float yaw; /*<  target yaw*/
 float yawspeed; /*<  target yawspeed*/
 uint8_t seq_debug; /*<  mavlink sequence*/
} mavlink_trajectory_setpoint_t;

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN 61
#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN 61
#define MAVLINK_MSG_ID_190_LEN 61
#define MAVLINK_MSG_ID_190_MIN_LEN 61

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_CRC 249
#define MAVLINK_MSG_ID_190_CRC 249



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_SETPOINT { \
    190, \
    "TRAJECTORY_SETPOINT", \
    16, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_trajectory_setpoint_t, timestamp) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_trajectory_setpoint_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_trajectory_setpoint_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_trajectory_setpoint_t, pos_z) }, \
         { "vel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_trajectory_setpoint_t, vel_x) }, \
         { "vel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_trajectory_setpoint_t, vel_y) }, \
         { "vel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_trajectory_setpoint_t, vel_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_trajectory_setpoint_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_trajectory_setpoint_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_trajectory_setpoint_t, acc_z) }, \
         { "jerk_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_trajectory_setpoint_t, jerk_x) }, \
         { "jerk_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_trajectory_setpoint_t, jerk_y) }, \
         { "jerk_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_trajectory_setpoint_t, jerk_z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_trajectory_setpoint_t, yaw) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_trajectory_setpoint_t, yawspeed) }, \
         { "seq_debug", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_trajectory_setpoint_t, seq_debug) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_SETPOINT { \
    "TRAJECTORY_SETPOINT", \
    16, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_trajectory_setpoint_t, timestamp) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_trajectory_setpoint_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_trajectory_setpoint_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_trajectory_setpoint_t, pos_z) }, \
         { "vel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_trajectory_setpoint_t, vel_x) }, \
         { "vel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_trajectory_setpoint_t, vel_y) }, \
         { "vel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_trajectory_setpoint_t, vel_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_trajectory_setpoint_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_trajectory_setpoint_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_trajectory_setpoint_t, acc_z) }, \
         { "jerk_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_trajectory_setpoint_t, jerk_x) }, \
         { "jerk_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_trajectory_setpoint_t, jerk_y) }, \
         { "jerk_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_trajectory_setpoint_t, jerk_z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_trajectory_setpoint_t, yaw) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_trajectory_setpoint_t, yawspeed) }, \
         { "seq_debug", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_trajectory_setpoint_t, seq_debug) }, \
         } \
}
#endif

/**
 * @brief Pack a trajectory_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time
 * @param pos_x  target position x
 * @param pos_y  target position y
 * @param pos_z  target position z
 * @param vel_x  target velocity x
 * @param vel_y  target velocity y
 * @param vel_z  target velocity z
 * @param acc_x  target acceleration x
 * @param acc_y  target acceleration y
 * @param acc_z  target acceleration z
 * @param jerk_x  target jerk x
 * @param jerk_y  target jerk y
 * @param jerk_z  target jerk z
 * @param yaw  target yaw
 * @param yawspeed  target yawspeed
 * @param seq_debug  mavlink sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t timestamp, float pos_x, float pos_y, float pos_z, float vel_x, float vel_y, float vel_z, float acc_x, float acc_y, float acc_z, float jerk_x, float jerk_y, float jerk_z, float yaw, float yawspeed, uint8_t seq_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, vel_x);
    _mav_put_float(buf, 20, vel_y);
    _mav_put_float(buf, 24, vel_z);
    _mav_put_float(buf, 28, acc_x);
    _mav_put_float(buf, 32, acc_y);
    _mav_put_float(buf, 36, acc_z);
    _mav_put_float(buf, 40, jerk_x);
    _mav_put_float(buf, 44, jerk_y);
    _mav_put_float(buf, 48, jerk_z);
    _mav_put_float(buf, 52, yaw);
    _mav_put_float(buf, 56, yawspeed);
    _mav_put_uint8_t(buf, 60, seq_debug);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN);
#else
    mavlink_trajectory_setpoint_t packet;
    packet.timestamp = timestamp;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.vel_x = vel_x;
    packet.vel_y = vel_y;
    packet.vel_z = vel_z;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.jerk_x = jerk_x;
    packet.jerk_y = jerk_y;
    packet.jerk_z = jerk_z;
    packet.yaw = yaw;
    packet.yawspeed = yawspeed;
    packet.seq_debug = seq_debug;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_CRC);
}

/**
 * @brief Pack a trajectory_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time
 * @param pos_x  target position x
 * @param pos_y  target position y
 * @param pos_z  target position z
 * @param vel_x  target velocity x
 * @param vel_y  target velocity y
 * @param vel_z  target velocity z
 * @param acc_x  target acceleration x
 * @param acc_y  target acceleration y
 * @param acc_z  target acceleration z
 * @param jerk_x  target jerk x
 * @param jerk_y  target jerk y
 * @param jerk_z  target jerk z
 * @param yaw  target yaw
 * @param yawspeed  target yawspeed
 * @param seq_debug  mavlink sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t timestamp,float pos_x,float pos_y,float pos_z,float vel_x,float vel_y,float vel_z,float acc_x,float acc_y,float acc_z,float jerk_x,float jerk_y,float jerk_z,float yaw,float yawspeed,uint8_t seq_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, vel_x);
    _mav_put_float(buf, 20, vel_y);
    _mav_put_float(buf, 24, vel_z);
    _mav_put_float(buf, 28, acc_x);
    _mav_put_float(buf, 32, acc_y);
    _mav_put_float(buf, 36, acc_z);
    _mav_put_float(buf, 40, jerk_x);
    _mav_put_float(buf, 44, jerk_y);
    _mav_put_float(buf, 48, jerk_z);
    _mav_put_float(buf, 52, yaw);
    _mav_put_float(buf, 56, yawspeed);
    _mav_put_uint8_t(buf, 60, seq_debug);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN);
#else
    mavlink_trajectory_setpoint_t packet;
    packet.timestamp = timestamp;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.vel_x = vel_x;
    packet.vel_y = vel_y;
    packet.vel_z = vel_z;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.jerk_x = jerk_x;
    packet.jerk_y = jerk_y;
    packet.jerk_z = jerk_z;
    packet.yaw = yaw;
    packet.yawspeed = yawspeed;
    packet.seq_debug = seq_debug;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_CRC);
}

/**
 * @brief Encode a trajectory_setpoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_trajectory_setpoint_t* trajectory_setpoint)
{
    return mavlink_msg_trajectory_setpoint_pack(system_id, component_id, msg, trajectory_setpoint->timestamp, trajectory_setpoint->pos_x, trajectory_setpoint->pos_y, trajectory_setpoint->pos_z, trajectory_setpoint->vel_x, trajectory_setpoint->vel_y, trajectory_setpoint->vel_z, trajectory_setpoint->acc_x, trajectory_setpoint->acc_y, trajectory_setpoint->acc_z, trajectory_setpoint->jerk_x, trajectory_setpoint->jerk_y, trajectory_setpoint->jerk_z, trajectory_setpoint->yaw, trajectory_setpoint->yawspeed, trajectory_setpoint->seq_debug);
}

/**
 * @brief Encode a trajectory_setpoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_trajectory_setpoint_t* trajectory_setpoint)
{
    return mavlink_msg_trajectory_setpoint_pack_chan(system_id, component_id, chan, msg, trajectory_setpoint->timestamp, trajectory_setpoint->pos_x, trajectory_setpoint->pos_y, trajectory_setpoint->pos_z, trajectory_setpoint->vel_x, trajectory_setpoint->vel_y, trajectory_setpoint->vel_z, trajectory_setpoint->acc_x, trajectory_setpoint->acc_y, trajectory_setpoint->acc_z, trajectory_setpoint->jerk_x, trajectory_setpoint->jerk_y, trajectory_setpoint->jerk_z, trajectory_setpoint->yaw, trajectory_setpoint->yawspeed, trajectory_setpoint->seq_debug);
}

/**
 * @brief Send a trajectory_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time
 * @param pos_x  target position x
 * @param pos_y  target position y
 * @param pos_z  target position z
 * @param vel_x  target velocity x
 * @param vel_y  target velocity y
 * @param vel_z  target velocity z
 * @param acc_x  target acceleration x
 * @param acc_y  target acceleration y
 * @param acc_z  target acceleration z
 * @param jerk_x  target jerk x
 * @param jerk_y  target jerk y
 * @param jerk_z  target jerk z
 * @param yaw  target yaw
 * @param yawspeed  target yawspeed
 * @param seq_debug  mavlink sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_trajectory_setpoint_send(mavlink_channel_t chan, uint32_t timestamp, float pos_x, float pos_y, float pos_z, float vel_x, float vel_y, float vel_z, float acc_x, float acc_y, float acc_z, float jerk_x, float jerk_y, float jerk_z, float yaw, float yawspeed, uint8_t seq_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, vel_x);
    _mav_put_float(buf, 20, vel_y);
    _mav_put_float(buf, 24, vel_z);
    _mav_put_float(buf, 28, acc_x);
    _mav_put_float(buf, 32, acc_y);
    _mav_put_float(buf, 36, acc_z);
    _mav_put_float(buf, 40, jerk_x);
    _mav_put_float(buf, 44, jerk_y);
    _mav_put_float(buf, 48, jerk_z);
    _mav_put_float(buf, 52, yaw);
    _mav_put_float(buf, 56, yawspeed);
    _mav_put_uint8_t(buf, 60, seq_debug);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT, buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_CRC);
#else
    mavlink_trajectory_setpoint_t packet;
    packet.timestamp = timestamp;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.vel_x = vel_x;
    packet.vel_y = vel_y;
    packet.vel_z = vel_z;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.jerk_x = jerk_x;
    packet.jerk_y = jerk_y;
    packet.jerk_z = jerk_z;
    packet.yaw = yaw;
    packet.yawspeed = yawspeed;
    packet.seq_debug = seq_debug;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT, (const char *)&packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_CRC);
#endif
}

/**
 * @brief Send a trajectory_setpoint message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_trajectory_setpoint_send_struct(mavlink_channel_t chan, const mavlink_trajectory_setpoint_t* trajectory_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_trajectory_setpoint_send(chan, trajectory_setpoint->timestamp, trajectory_setpoint->pos_x, trajectory_setpoint->pos_y, trajectory_setpoint->pos_z, trajectory_setpoint->vel_x, trajectory_setpoint->vel_y, trajectory_setpoint->vel_z, trajectory_setpoint->acc_x, trajectory_setpoint->acc_y, trajectory_setpoint->acc_z, trajectory_setpoint->jerk_x, trajectory_setpoint->jerk_y, trajectory_setpoint->jerk_z, trajectory_setpoint->yaw, trajectory_setpoint->yawspeed, trajectory_setpoint->seq_debug);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT, (const char *)trajectory_setpoint, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_trajectory_setpoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t timestamp, float pos_x, float pos_y, float pos_z, float vel_x, float vel_y, float vel_z, float acc_x, float acc_y, float acc_z, float jerk_x, float jerk_y, float jerk_z, float yaw, float yawspeed, uint8_t seq_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, vel_x);
    _mav_put_float(buf, 20, vel_y);
    _mav_put_float(buf, 24, vel_z);
    _mav_put_float(buf, 28, acc_x);
    _mav_put_float(buf, 32, acc_y);
    _mav_put_float(buf, 36, acc_z);
    _mav_put_float(buf, 40, jerk_x);
    _mav_put_float(buf, 44, jerk_y);
    _mav_put_float(buf, 48, jerk_z);
    _mav_put_float(buf, 52, yaw);
    _mav_put_float(buf, 56, yawspeed);
    _mav_put_uint8_t(buf, 60, seq_debug);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT, buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_CRC);
#else
    mavlink_trajectory_setpoint_t *packet = (mavlink_trajectory_setpoint_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;
    packet->pos_z = pos_z;
    packet->vel_x = vel_x;
    packet->vel_y = vel_y;
    packet->vel_z = vel_z;
    packet->acc_x = acc_x;
    packet->acc_y = acc_y;
    packet->acc_z = acc_z;
    packet->jerk_x = jerk_x;
    packet->jerk_y = jerk_y;
    packet->jerk_z = jerk_z;
    packet->yaw = yaw;
    packet->yawspeed = yawspeed;
    packet->seq_debug = seq_debug;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT, (const char *)packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_CRC);
#endif
}
#endif

#endif

// MESSAGE TRAJECTORY_SETPOINT UNPACKING


/**
 * @brief Get field timestamp from trajectory_setpoint message
 *
 * @return  Time
 */
static inline uint32_t mavlink_msg_trajectory_setpoint_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field pos_x from trajectory_setpoint message
 *
 * @return  target position x
 */
static inline float mavlink_msg_trajectory_setpoint_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pos_y from trajectory_setpoint message
 *
 * @return  target position y
 */
static inline float mavlink_msg_trajectory_setpoint_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pos_z from trajectory_setpoint message
 *
 * @return  target position z
 */
static inline float mavlink_msg_trajectory_setpoint_get_pos_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vel_x from trajectory_setpoint message
 *
 * @return  target velocity x
 */
static inline float mavlink_msg_trajectory_setpoint_get_vel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_y from trajectory_setpoint message
 *
 * @return  target velocity y
 */
static inline float mavlink_msg_trajectory_setpoint_get_vel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vel_z from trajectory_setpoint message
 *
 * @return  target velocity z
 */
static inline float mavlink_msg_trajectory_setpoint_get_vel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field acc_x from trajectory_setpoint message
 *
 * @return  target acceleration x
 */
static inline float mavlink_msg_trajectory_setpoint_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field acc_y from trajectory_setpoint message
 *
 * @return  target acceleration y
 */
static inline float mavlink_msg_trajectory_setpoint_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field acc_z from trajectory_setpoint message
 *
 * @return  target acceleration z
 */
static inline float mavlink_msg_trajectory_setpoint_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field jerk_x from trajectory_setpoint message
 *
 * @return  target jerk x
 */
static inline float mavlink_msg_trajectory_setpoint_get_jerk_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field jerk_y from trajectory_setpoint message
 *
 * @return  target jerk y
 */
static inline float mavlink_msg_trajectory_setpoint_get_jerk_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field jerk_z from trajectory_setpoint message
 *
 * @return  target jerk z
 */
static inline float mavlink_msg_trajectory_setpoint_get_jerk_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field yaw from trajectory_setpoint message
 *
 * @return  target yaw
 */
static inline float mavlink_msg_trajectory_setpoint_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field yawspeed from trajectory_setpoint message
 *
 * @return  target yawspeed
 */
static inline float mavlink_msg_trajectory_setpoint_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field seq_debug from trajectory_setpoint message
 *
 * @return  mavlink sequence
 */
static inline uint8_t mavlink_msg_trajectory_setpoint_get_seq_debug(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Decode a trajectory_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param trajectory_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_trajectory_setpoint_decode(const mavlink_message_t* msg, mavlink_trajectory_setpoint_t* trajectory_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    trajectory_setpoint->timestamp = mavlink_msg_trajectory_setpoint_get_timestamp(msg);
    trajectory_setpoint->pos_x = mavlink_msg_trajectory_setpoint_get_pos_x(msg);
    trajectory_setpoint->pos_y = mavlink_msg_trajectory_setpoint_get_pos_y(msg);
    trajectory_setpoint->pos_z = mavlink_msg_trajectory_setpoint_get_pos_z(msg);
    trajectory_setpoint->vel_x = mavlink_msg_trajectory_setpoint_get_vel_x(msg);
    trajectory_setpoint->vel_y = mavlink_msg_trajectory_setpoint_get_vel_y(msg);
    trajectory_setpoint->vel_z = mavlink_msg_trajectory_setpoint_get_vel_z(msg);
    trajectory_setpoint->acc_x = mavlink_msg_trajectory_setpoint_get_acc_x(msg);
    trajectory_setpoint->acc_y = mavlink_msg_trajectory_setpoint_get_acc_y(msg);
    trajectory_setpoint->acc_z = mavlink_msg_trajectory_setpoint_get_acc_z(msg);
    trajectory_setpoint->jerk_x = mavlink_msg_trajectory_setpoint_get_jerk_x(msg);
    trajectory_setpoint->jerk_y = mavlink_msg_trajectory_setpoint_get_jerk_y(msg);
    trajectory_setpoint->jerk_z = mavlink_msg_trajectory_setpoint_get_jerk_z(msg);
    trajectory_setpoint->yaw = mavlink_msg_trajectory_setpoint_get_yaw(msg);
    trajectory_setpoint->yawspeed = mavlink_msg_trajectory_setpoint_get_yawspeed(msg);
    trajectory_setpoint->seq_debug = mavlink_msg_trajectory_setpoint_get_seq_debug(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN? msg->len : MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN;
        memset(trajectory_setpoint, 0, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_LEN);
    memcpy(trajectory_setpoint, _MAV_PAYLOAD(msg), len);
#endif
}
