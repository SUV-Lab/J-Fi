/** @file
 *  @brief MAVLink comm protocol generated from rover.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_ROVER_H
#define MAVLINK_ROVER_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_ROVER.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_ROVER_XML_HASH 3462387848964698199

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{190, 249, 61, 61, 0, 0, 0}, {191, 65, 19, 19, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_ROVER

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_trajectory_setpoint.h"
#include "./mavlink_msg_vehicle_status.h"

// base include



#if MAVLINK_ROVER_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_TRAJECTORY_SETPOINT, MAVLINK_MESSAGE_INFO_VEHICLE_STATUS}
# define MAVLINK_MESSAGE_NAMES {{ "TRAJECTORY_SETPOINT", 190 }, { "VEHICLE_STATUS", 191 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_ROVER_H
