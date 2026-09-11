#include "cran_hil_protocol.h"

#include <algorithm>
#include <limits>

namespace {

constexpr std::size_t kHeaderSize = 24;
constexpr std::size_t kCrcSize = 4;
constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::uint8_t kMagic[4] = {'C', 'R', 'H', 'L'};

void write_u16(std::uint8_t* dst, std::uint16_t value) {
    dst[0] = static_cast<std::uint8_t>(value);
    dst[1] = static_cast<std::uint8_t>(value >> 8);
}

void write_u32(std::uint8_t* dst, std::uint32_t value) {
    for (unsigned index = 0; index < 4; ++index) {
        dst[index] = static_cast<std::uint8_t>(value >> (index * 8));
    }
}

void write_u64(std::uint8_t* dst, std::uint64_t value) {
    for (unsigned index = 0; index < 8; ++index) {
        dst[index] = static_cast<std::uint8_t>(value >> (index * 8));
    }
}

std::uint16_t read_u16(const std::uint8_t* src) {
    return static_cast<std::uint16_t>(src[0]) |
        (static_cast<std::uint16_t>(src[1]) << 8);
}

std::uint32_t read_u32(const std::uint8_t* src) {
    std::uint32_t value = 0;
    for (unsigned index = 0; index < 4; ++index) {
        value |= static_cast<std::uint32_t>(src[index]) << (index * 8);
    }
    return value;
}

std::uint64_t read_u64(const std::uint8_t* src) {
    std::uint64_t value = 0;
    for (unsigned index = 0; index < 8; ++index) {
        value |= static_cast<std::uint64_t>(src[index]) << (index * 8);
    }
    return value;
}

}  // namespace

extern "C" {

std::uint32_t cran_hil_crc32(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr && length != 0) {
        return 0;
    }
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (unsigned bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask =
                0u - static_cast<std::uint32_t>(crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

int cran_hil_encode(
    std::uint8_t message_type,
    std::uint16_t flags,
    std::uint32_t sequence,
    std::uint64_t timestamp_ns,
    const std::uint8_t* payload,
    std::uint32_t payload_length,
    std::uint8_t* destination,
    std::size_t capacity) {
    if ((payload == nullptr && payload_length != 0) || destination == nullptr) {
        return 0;
    }
    const std::size_t required =
        kHeaderSize + static_cast<std::size_t>(payload_length) + kCrcSize;
    if (required > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return 0;
    }
    if (capacity < required) {
        return -static_cast<int>(required);
    }
    std::copy(kMagic, kMagic + 4, destination);
    destination[4] = kProtocolVersion;
    destination[5] = message_type;
    write_u16(destination + 6, flags);
    write_u32(destination + 8, sequence);
    write_u64(destination + 12, timestamp_ns);
    write_u32(destination + 20, payload_length);
    if (payload_length != 0) {
        std::copy(payload, payload + payload_length, destination + kHeaderSize);
    }
    write_u32(
        destination + required - kCrcSize,
        cran_hil_crc32(destination, required - kCrcSize));
    return static_cast<int>(required);
}

int cran_hil_decode(
    const std::uint8_t* data,
    std::size_t length,
    cran_hil_frame_view* destination) {
    if (data == nullptr || destination == nullptr ||
        length < kHeaderSize + kCrcSize) {
        return -1;
    }
    if (!std::equal(kMagic, kMagic + 4, data)) {
        return -2;
    }
    if (data[4] != kProtocolVersion) {
        return -3;
    }
    const std::uint32_t payload_length = read_u32(data + 20);
    const std::size_t expected =
        kHeaderSize + static_cast<std::size_t>(payload_length) + kCrcSize;
    if (length != expected) {
        return -4;
    }
    const std::uint32_t expected_crc = read_u32(data + length - kCrcSize);
    if (cran_hil_crc32(data, length - kCrcSize) != expected_crc) {
        return -5;
    }
    destination->message_type = data[5];
    destination->flags = read_u16(data + 6);
    destination->sequence = read_u32(data + 8);
    destination->timestamp_ns = read_u64(data + 12);
    destination->payload = data + kHeaderSize;
    destination->payload_length = payload_length;
    return 0;
}

}  // extern "C"
