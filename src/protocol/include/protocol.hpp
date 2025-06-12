#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

// -----------------------------------------------------------------------------
// protocol.hpp
// -----------------------------------------------------------------------------
// Frame format (little‑endian)
//   +-----------+------+-----+-------+--------+---------+---------+
//   |  Magic    | LEN  |SEQ  | SYSID | MSGID  | Payload | CRC‑16  |
//   | 0xAA 0x01 | 1 B  | 1 B | 1  B  | 1  B   | 0‑255 B | 2  B    |
//   +-----------+------+-----+-------+--------+---------+---------+
//   LEN  : number of payload bytes (CRC not included)
//   CRC  : CRC‑16/X.25 over [LEN..Payload]
//
// -----------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <vector>

namespace protocol {

// -----------------------------------------------------------------------------
// 16‑bit CRC (X25 = CRC‑16/ITU‑X.25, poly 0x1021, init 0xFFFF)
// -----------------------------------------------------------------------------
uint16_t crc16_x25(const uint8_t* data, size_t len);

// -----------------------------------------------------------------------------
// Packet header (positions after the two‑byte magic field)
// -----------------------------------------------------------------------------
struct PacketHeader {
    uint8_t len;   // payload length (0–255)
    uint8_t seq;   // sequence number (wraps at 255)
    uint8_t sysid; // sender node ID
    uint8_t msgid; // message ID
};

// -----------------------------------------------------------------------------
// Protocol class: assembles / dissects frames byte‑by‑byte
// -----------------------------------------------------------------------------
class Protocol {
public:
    // magic bytes & constants
    static constexpr uint8_t MAGIC1      = 0xAA;
    static constexpr uint8_t MAGIC2      = 0x01;   // protocol version 1
    static constexpr size_t  HEADER_SIZE = 2 /*magic*/ + sizeof(PacketHeader);
    static constexpr size_t  CRC_SIZE    = 2;
    static constexpr size_t  MAX_PAYLOAD = 255;

    Protocol();

    // ---------------------------------------------------------------------
    // Serialize a packet
    //  seq   : monotonically increasing sequence number
    //  sysid : sender drone ID
    //  msgid : application‑defined message ID
    //  payload: binary payload (0–255 bytes)
    // ---------------------------------------------------------------------
    static std::vector<uint8_t> createPacket(uint8_t seq,
                                             uint8_t sysid,
                                             uint8_t msgid,
                                             const std::vector<uint8_t>& payload);

    // ---------------------------------------------------------------------
    // Incremental parser – feed one byte at a time.
    // Returns true when a complete & valid frame is assembled.
    // After true, seq()/sysid()/msgid()/payload() hold the data.
    // ---------------------------------------------------------------------
    bool parseByte(uint8_t byte);

    // getters (valid only if last parseByte() returned true)
    uint8_t                   seq()     const { return hdr_.seq;   }
    uint8_t                   sysid()   const { return hdr_.sysid; }
    uint8_t                   msgid()   const { return hdr_.msgid; }
    const std::vector<uint8_t>& payload() const { return payload_cache_; }

    // resets internal state machine so a new stream can be parsed
    void reset();

private:
    enum class State {
        WAIT_MAGIC1,
        WAIT_MAGIC2,
        WAIT_LEN,
        WAIT_HEADER,
        WAIT_PAYLOAD,
        WAIT_CRC_LSB,
        WAIT_CRC_MSB
    } state_ { State::WAIT_MAGIC1 };

    bool validateCRC();

    // parser state
    std::vector<uint8_t> buffer_;
    size_t   expected_  = 0;      // total bytes expected for current frame
    uint16_t crc_recv_  = 0;      // CRC extracted from stream
    bool     complete_  = false;  // true when a frame is complete & valid

    PacketHeader hdr_{};                 // parsed header (without magic)
    std::vector<uint8_t> payload_cache_; // cached payload after success
};

} // namespace protocol

#endif // PROTOCOL_HPP
