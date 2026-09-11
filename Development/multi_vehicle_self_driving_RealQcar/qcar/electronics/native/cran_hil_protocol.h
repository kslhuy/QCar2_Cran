#pragma once

#include "cran_core.h"

#include <cstddef>
#include <cstdint>

// Allocation-free CRHL framing shared with electronics/hil.py. Multibyte
// fields are explicitly little-endian; callers must reassemble transport-level
// fragments before decode (for example CAN-FD segmentation).

extern "C" {

enum cran_hil_message_type : std::uint8_t {
    CRAN_HIL_HELLO = 1,
    CRAN_HIL_CAPABILITIES = 2,
    CRAN_HIL_NAV_SENSOR = 3,
    CRAN_HIL_VEHICLE_INPUT = 4,
    CRAN_HIL_V2V_INPUT = 5,
    CRAN_HIL_STEP = 6,
    CRAN_HIL_VEHICLE_OUTPUT = 7,
    CRAN_HIL_V2V_OUTPUT = 8,
    CRAN_HIL_STATUS = 9,
    CRAN_HIL_RESET = 10,
    CRAN_HIL_ERROR = 11,
};

struct cran_hil_frame_view {
    std::uint8_t message_type;
    std::uint16_t flags;
    std::uint32_t sequence;
    std::uint64_t timestamp_ns;
    const std::uint8_t* payload;
    std::uint32_t payload_length;
};

CRAN_API std::uint32_t cran_hil_crc32(
    const std::uint8_t* data, std::size_t length);

// Returns encoded length, a negative required capacity, or zero for bad input.
CRAN_API int cran_hil_encode(
    std::uint8_t message_type,
    std::uint16_t flags,
    std::uint32_t sequence,
    std::uint64_t timestamp_ns,
    const std::uint8_t* payload,
    std::uint32_t payload_length,
    std::uint8_t* destination,
    std::size_t capacity);

// Returns 0 on success; negative values identify framing/version/CRC errors.
// The payload view points inside `data` and remains valid only while `data` is.
CRAN_API int cran_hil_decode(
    const std::uint8_t* data,
    std::size_t length,
    cran_hil_frame_view* destination);

}
