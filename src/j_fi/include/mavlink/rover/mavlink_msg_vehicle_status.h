#pragma once
// MESSAGE VEHICLE_STATUS PACKING

#define MAVLINK_MSG_ID_VEHICLE_STATUS 191


typedef struct __mavlink_vehicle_status_t {
 uint64_t timestamp; /*<  Time since system start (microseconds)*/
 uint64_t armed_time; /*<  Arming timestamp (microseconds)*/
 uint8_t arming_state; /*<  Current arming state*/
 uint8_t nav_state; /*<  Currently active navigation mode*/
 uint8_t seq_debug; /*<  mavlink sequence*/
} mavlink_vehicle_status_t;

#define MAVLINK_MSG_ID_VEHICLE_STATUS_LEN 19
#define MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN 19
#define MAVLINK_MSG_ID_191_LEN 19
#define MAVLINK_MSG_ID_191_MIN_LEN 19

#define MAVLINK_MSG_ID_VEHICLE_STATUS_CRC 65
#define MAVLINK_MSG_ID_191_CRC 65



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VEHICLE_STATUS { \
    191, \
    "VEHICLE_STATUS", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vehicle_status_t, timestamp) }, \
         { "armed_time", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_vehicle_status_t, armed_time) }, \
         { "arming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_vehicle_status_t, arming_state) }, \
         { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_vehicle_status_t, nav_state) }, \
         { "seq_debug", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_vehicle_status_t, seq_debug) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VEHICLE_STATUS { \
    "VEHICLE_STATUS", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vehicle_status_t, timestamp) }, \
         { "armed_time", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_vehicle_status_t, armed_time) }, \
         { "arming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_vehicle_status_t, arming_state) }, \
         { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_vehicle_status_t, nav_state) }, \
         { "seq_debug", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_vehicle_status_t, seq_debug) }, \
         } \
}
#endif

/**
 * @brief Pack a vehicle_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time since system start (microseconds)
 * @param armed_time  Arming timestamp (microseconds)
 * @param arming_state  Current arming state
 * @param nav_state  Currently active navigation mode
 * @param seq_debug  mavlink sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t armed_time, uint8_t arming_state, uint8_t nav_state, uint8_t seq_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, armed_time);
    _mav_put_uint8_t(buf, 16, arming_state);
    _mav_put_uint8_t(buf, 17, nav_state);
    _mav_put_uint8_t(buf, 18, seq_debug);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
#else
    mavlink_vehicle_status_t packet;
    packet.timestamp = timestamp;
    packet.armed_time = armed_time;
    packet.arming_state = arming_state;
    packet.nav_state = nav_state;
    packet.seq_debug = seq_debug;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VEHICLE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
}

/**
 * @brief Pack a vehicle_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time since system start (microseconds)
 * @param armed_time  Arming timestamp (microseconds)
 * @param arming_state  Current arming state
 * @param nav_state  Currently active navigation mode
 * @param seq_debug  mavlink sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint64_t armed_time,uint8_t arming_state,uint8_t nav_state,uint8_t seq_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, armed_time);
    _mav_put_uint8_t(buf, 16, arming_state);
    _mav_put_uint8_t(buf, 17, nav_state);
    _mav_put_uint8_t(buf, 18, seq_debug);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
#else
    mavlink_vehicle_status_t packet;
    packet.timestamp = timestamp;
    packet.armed_time = armed_time;
    packet.arming_state = arming_state;
    packet.nav_state = nav_state;
    packet.seq_debug = seq_debug;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VEHICLE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
}

/**
 * @brief Encode a vehicle_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vehicle_status_t* vehicle_status)
{
    return mavlink_msg_vehicle_status_pack(system_id, component_id, msg, vehicle_status->timestamp, vehicle_status->armed_time, vehicle_status->arming_state, vehicle_status->nav_state, vehicle_status->seq_debug);
}

/**
 * @brief Encode a vehicle_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vehicle_status_t* vehicle_status)
{
    return mavlink_msg_vehicle_status_pack_chan(system_id, component_id, chan, msg, vehicle_status->timestamp, vehicle_status->armed_time, vehicle_status->arming_state, vehicle_status->nav_state, vehicle_status->seq_debug);
}

/**
 * @brief Send a vehicle_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time since system start (microseconds)
 * @param armed_time  Arming timestamp (microseconds)
 * @param arming_state  Current arming state
 * @param nav_state  Currently active navigation mode
 * @param seq_debug  mavlink sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vehicle_status_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t armed_time, uint8_t arming_state, uint8_t nav_state, uint8_t seq_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, armed_time);
    _mav_put_uint8_t(buf, 16, arming_state);
    _mav_put_uint8_t(buf, 17, nav_state);
    _mav_put_uint8_t(buf, 18, seq_debug);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, buf, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#else
    mavlink_vehicle_status_t packet;
    packet.timestamp = timestamp;
    packet.armed_time = armed_time;
    packet.arming_state = arming_state;
    packet.nav_state = nav_state;
    packet.seq_debug = seq_debug;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#endif
}

/**
 * @brief Send a vehicle_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vehicle_status_send_struct(mavlink_channel_t chan, const mavlink_vehicle_status_t* vehicle_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vehicle_status_send(chan, vehicle_status->timestamp, vehicle_status->armed_time, vehicle_status->arming_state, vehicle_status->nav_state, vehicle_status->seq_debug);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, (const char *)vehicle_status, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_VEHICLE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vehicle_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t armed_time, uint8_t arming_state, uint8_t nav_state, uint8_t seq_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, armed_time);
    _mav_put_uint8_t(buf, 16, arming_state);
    _mav_put_uint8_t(buf, 17, nav_state);
    _mav_put_uint8_t(buf, 18, seq_debug);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, buf, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#else
    mavlink_vehicle_status_t *packet = (mavlink_vehicle_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->armed_time = armed_time;
    packet->arming_state = arming_state;
    packet->nav_state = nav_state;
    packet->seq_debug = seq_debug;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_STATUS, (const char *)packet, MAVLINK_MSG_ID_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_VEHICLE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE VEHICLE_STATUS UNPACKING


/**
 * @brief Get field timestamp from vehicle_status message
 *
 * @return  Time since system start (microseconds)
 */
static inline uint64_t mavlink_msg_vehicle_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field armed_time from vehicle_status message
 *
 * @return  Arming timestamp (microseconds)
 */
static inline uint64_t mavlink_msg_vehicle_status_get_armed_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field arming_state from vehicle_status message
 *
 * @return  Current arming state
 */
static inline uint8_t mavlink_msg_vehicle_status_get_arming_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field nav_state from vehicle_status message
 *
 * @return  Currently active navigation mode
 */
static inline uint8_t mavlink_msg_vehicle_status_get_nav_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field seq_debug from vehicle_status message
 *
 * @return  mavlink sequence
 */
static inline uint8_t mavlink_msg_vehicle_status_get_seq_debug(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Decode a vehicle_status message into a struct
 *
 * @param msg The message to decode
 * @param vehicle_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_vehicle_status_decode(const mavlink_message_t* msg, mavlink_vehicle_status_t* vehicle_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vehicle_status->timestamp = mavlink_msg_vehicle_status_get_timestamp(msg);
    vehicle_status->armed_time = mavlink_msg_vehicle_status_get_armed_time(msg);
    vehicle_status->arming_state = mavlink_msg_vehicle_status_get_arming_state(msg);
    vehicle_status->nav_state = mavlink_msg_vehicle_status_get_nav_state(msg);
    vehicle_status->seq_debug = mavlink_msg_vehicle_status_get_seq_debug(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VEHICLE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_VEHICLE_STATUS_LEN;
        memset(vehicle_status, 0, MAVLINK_MSG_ID_VEHICLE_STATUS_LEN);
    memcpy(vehicle_status, _MAV_PAYLOAD(msg), len);
#endif
}
