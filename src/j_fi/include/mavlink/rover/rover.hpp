/** @file
 *	@brief MAVLink comm protocol generated from rover.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace rover {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (through @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 2> MESSAGE_ENTRIES {{ {190, 202, 60, 60, 0, 0, 0}, {191, 111, 18, 18, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS




} // namespace rover
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_trajectory_setpoint.hpp"
#include "./mavlink_msg_vehicle_status.hpp"

// base include

