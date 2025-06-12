// -----------------------------------------------------------------------------
// protocol.cpp – Implementation file
// -----------------------------------------------------------------------------
#include "protocol.hpp"

#include <cstring>   // std::memset
#include <stdexcept> // std::runtime_error

namespace protocol {

// -----------------------------------------------------------------------------
// CRC‑16/X25 implementation (bit‑wise, table‑less)
// -----------------------------------------------------------------------------
uint16_t crc16_x25(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc & 0xFFFF; // final XOR 0x0000 (X25 variant)
}

// -----------------------------------------------------------------------------
// ctor / reset helpers
// -----------------------------------------------------------------------------
Protocol::Protocol() { reset(); }

void Protocol::reset()
{
    state_   = State::WAIT_MAGIC1;
    buffer_.clear();
    expected_ = 0;
    crc_recv_ = 0;
    complete_ = false;
    payload_cache_.clear();
    std::memset(&hdr_, 0, sizeof(hdr_));
}

// -----------------------------------------------------------------------------
// createPacket – static
// -----------------------------------------------------------------------------
std::vector<uint8_t> Protocol::createPacket(uint8_t seq,
                                           uint8_t sysid,
                                           uint8_t msgid,
                                           const std::vector<uint8_t>& payload)
{
    if (payload.size() > MAX_PAYLOAD) {
        throw std::runtime_error("Payload too large (max 255)");
    }

    std::vector<uint8_t> pkt;
    pkt.reserve(HEADER_SIZE + payload.size() + CRC_SIZE);

    // 1) magic bytes
    pkt.push_back(MAGIC1);
    pkt.push_back(MAGIC2);

    // 2) header
    PacketHeader hdr;
    hdr.len   = static_cast<uint8_t>(payload.size());
    hdr.seq   = seq;
    hdr.sysid = sysid;
    hdr.msgid = msgid;
    const uint8_t* h = reinterpret_cast<const uint8_t*>(&hdr);
    pkt.insert(pkt.end(), h, h + sizeof(PacketHeader));

    // 3) payload
    pkt.insert(pkt.end(), payload.begin(), payload.end());

    // 4) CRC over [LEN .. last payload byte]
    uint16_t crc = crc16_x25(pkt.data() + 2, sizeof(PacketHeader) + payload.size());
    pkt.push_back(static_cast<uint8_t>(crc & 0xFF)); // LSB first
    pkt.push_back(static_cast<uint8_t>(crc >> 8));   // MSB

    return pkt;
}

// -----------------------------------------------------------------------------
// parseByte – incremental FSM parser
// -----------------------------------------------------------------------------
bool Protocol::parseByte(uint8_t byte)
{
    switch (state_) {
    case State::WAIT_MAGIC1:
        if (byte == MAGIC1) {
            reset();
            buffer_.push_back(byte);
            state_ = State::WAIT_MAGIC2;
        }
        break;

    case State::WAIT_MAGIC2:
        if (byte == MAGIC2) {
            buffer_.push_back(byte);
            state_ = State::WAIT_LEN;
        } else {
            state_ = State::WAIT_MAGIC1; // resync
        }
        break;

    case State::WAIT_LEN:
        hdr_.len = byte;
        buffer_.push_back(byte);
        state_ = State::WAIT_HEADER;
        break;

    case State::WAIT_HEADER:
        buffer_.push_back(byte);
        if (buffer_.size() == HEADER_SIZE) {
            // copy remaining header bytes (after LEN field)
            const uint8_t* p = buffer_.data() + 3; // magic(2)+len(1)
            hdr_.seq   = p[0];
            hdr_.sysid = p[1];
            hdr_.msgid = p[2];

            expected_ = HEADER_SIZE + hdr_.len + CRC_SIZE;
            if (expected_ < HEADER_SIZE || expected_ > HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE) {
                state_ = State::WAIT_MAGIC1; // invalid length, restart
            } else {
                state_ = (hdr_.len == 0) ? State::WAIT_CRC_LSB : State::WAIT_PAYLOAD;
            }
        }
        break;

    case State::WAIT_PAYLOAD:
        buffer_.push_back(byte);
        if (buffer_.size() == HEADER_SIZE + hdr_.len) {
            state_ = State::WAIT_CRC_LSB;
        }
        break;

    case State::WAIT_CRC_LSB:
        buffer_.push_back(byte);
        crc_recv_ = byte;
        state_ = State::WAIT_CRC_MSB;
        break;

    case State::WAIT_CRC_MSB:
        buffer_.push_back(byte);
        crc_recv_ |= static_cast<uint16_t>(byte) << 8;
        if (validateCRC()) {
            complete_ = true;
        }
        state_ = State::WAIT_MAGIC1; // ready for next frame
        break;
    }
    return complete_;
}

// -----------------------------------------------------------------------------
// validateCRC – private helper
// -----------------------------------------------------------------------------
bool Protocol::validateCRC()
{
    if (buffer_.size() != expected_) return false;

    uint16_t crc_calc = crc16_x25(buffer_.data() + 2, expected_ - 2 - CRC_SIZE);
    if (crc_calc != crc_recv_) return false;

    // CRC OK → cache payload
    payload_cache_.assign(buffer_.begin() + HEADER_SIZE,
                          buffer_.begin() + HEADER_SIZE + hdr_.len);
    return true;
}

} // namespace protocol
