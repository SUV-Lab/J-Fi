#include "gmcs_proto.hpp"
#include <gtest/gtest.h>
#include <iostream>

using gmcs_proto_lib::PacketHeader;

/**
 * @brief Helper function to compute 16-bit XOR checksum.
 */
uint16_t computeChecksum(const std::vector<uint8_t>& packet)
{
    uint16_t checksum = 0;
    for (size_t i = 0; i < packet.size() - 2; i++) { // Exclude last 2 bytes (checksum)
        checksum ^= packet[i];
    }
    return checksum;
}

/**
 * @brief Test for parsing a valid packet.
 */
TEST(GMCSProtocolTest, ParseValidPacket)
{
    std::vector<uint8_t> packet = {
        0xAA, 0x55, 16, 1, 2, 3, 4, 5, 6, 7,  // Header (10 bytes)
        0x12, 0x34, 0x56, 0x78,  // Data (4 bytes)
        0x00, 0x00               // Checksum (2 bytes, placeholder)
    };

    // Compute and insert 16-bit checksum
    uint16_t checksum = computeChecksum(packet);
    packet[packet.size() - 2] = static_cast<uint8_t>(checksum & 0xFF); // Lower byte
    packet[packet.size() - 1] = static_cast<uint8_t>((checksum >> 8) & 0xFF); // Upper byte

    gmcs_proto_lib::GMCSProtocol parser;
    bool success = false;

    for (auto byte : packet) {
        success = gmcs_proto_lib::GMCSProtocol::parse(byte, parser);
    }

    // Ensure parsing was successful
    ASSERT_TRUE(success);
}

/**
 * @brief Test for handling an incorrect checksum.
 */
TEST(GMCSProtocolTest, ParseInvalidChecksum)
{
    std::vector<uint8_t> packet = {
        0xAA, 0x55, 16, 1, 2, 3, 4, 5, 6, 7,  // Header (10 bytes)
        0x12, 0x34, 0x56, 0x78,  // Data (4 bytes)
        0x99, 0x99               // Incorrect Checksum
    };

    gmcs_proto_lib::GMCSProtocol parser;
    bool success = false;

    for (auto byte : packet) {
        success = gmcs_proto_lib::GMCSProtocol::parse(byte, parser);
    }

    // Parsing should fail due to incorrect checksum
    ASSERT_FALSE(success);
}

/**
 * @brief Test for handling an incomplete packet.
 */
TEST(GMCSProtocolTest, ParseIncompletePacket)
{
    std::vector<uint8_t> incomplete_packet = {
        0xAA, 0x55, 16, 1, 2, 3, 4, 5  // Only part of the header (less than expected)
    };

    gmcs_proto_lib::GMCSProtocol parser;
    bool success = false;

    for (auto byte : incomplete_packet) {
        success = gmcs_proto_lib::GMCSProtocol::parse(byte, parser);
    }

    // Parsing should fail because the packet is incomplete
    ASSERT_FALSE(success);
}

/**
 * @brief Test for resetting after a failed packet.
 */
TEST(GMCSProtocolTest, ResetAfterInvalidPacket)
{
    std::vector<uint8_t> invalid_packet = {
        0xAA, 0x55, 16, 1, 2, 3, 4, 5, 6, 7,   // Header (10 bytes)
        0x12, 0x34, 0x56, 0x78,  // Data (4 bytes)
        0x99, 0x99               // Incorrect Checksum
    };

    std::vector<uint8_t> valid_packet = {
        0xAA, 0x55, 16, 1, 2, 3, 4, 5, 6, 7,  // Header (10 bytes)
        0x12, 0x34, 0x56, 0x78,  // Data (4 bytes)
        0x00, 0x00               // Checksum (placeholder)
    };

    // Compute and insert correct checksum for the valid packet
    uint16_t checksum = computeChecksum(valid_packet);
    valid_packet[valid_packet.size() - 2] = static_cast<uint8_t>(checksum & 0xFF);
    valid_packet[valid_packet.size() - 1] = static_cast<uint8_t>((checksum >> 8) & 0xFF);

    gmcs_proto_lib::GMCSProtocol parser;
    bool success = false;

    // First, send an invalid packet
    for (auto byte : invalid_packet) {
        success = gmcs_proto_lib::GMCSProtocol::parse(byte, parser);
    }

    // Parsing should fail due to incorrect checksum
    ASSERT_FALSE(success);

    // Now, send a valid packet
    for (auto byte : valid_packet) {
        success = gmcs_proto_lib::GMCSProtocol::parse(byte, parser);
    }

    // Ensure parsing succeeds after failure
    ASSERT_TRUE(success);
}

/**
 * @brief Test for creating a valid GMCSProtocol packet.
 */
TEST(GMCSProtocolTest, CreatePacket)
{
    std::vector<uint8_t> data = {1, 2, 3, 4, 5, 6, 7, 8};

    std::vector<uint8_t> packet = gmcs_proto_lib::GMCSProtocol::createPacket(data);

    // Verify packet structure
    ASSERT_EQ(packet[0], 0xAA); // SYNC1
    ASSERT_EQ(packet[1], 0x55); // SYNC2

    // Validate length
    uint8_t expected_length = sizeof(PacketHeader) + data.size() + 2;
    ASSERT_EQ(packet[2], expected_length);

    // Ensure data is placed correctly
    std::vector<uint8_t> extracted_data(packet.begin() + sizeof(PacketHeader), packet.end() - 2);
    ASSERT_EQ(extracted_data, data);
}

/**
 * @brief Main function for running tests.
 */
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
