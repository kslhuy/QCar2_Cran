#pragma once

#include "cran_core.h"

#include <cstddef>
#include <cstdint>

// Vendor-neutral adapter between the algorithm core and a board support
// package. The application supplies a monotonic clock and an output callback;
// no STM32, RTOS or Linux API is referenced by this interface.

extern "C" {

enum cran_target_channel : std::uint8_t {
    CRAN_TARGET_VEHICLE_CHANNEL = 0,
    CRAN_TARGET_V2V_CHANNEL = 1,
};

enum cran_target_feature : std::uint32_t {
    CRAN_FEATURE_FIRMWARE_CORE = 1u << 0,
    CRAN_FEATURE_TRUST = 1u << 1,
    CRAN_FEATURE_OBSERVER = 1u << 2,
    CRAN_FEATURE_V2V = 1u << 3,
    CRAN_FEATURE_SENSOR_ACQUISITION = 1u << 4,
};

enum cran_target_transport : std::uint32_t {
    CRAN_TRANSPORT_UART = 1u << 0,
    CRAN_TRANSPORT_SPI = 1u << 1,
    CRAN_TRANSPORT_CAN = 1u << 2,
    CRAN_TRANSPORT_CAN_FD = 1u << 3,
    CRAN_TRANSPORT_ETHERNET = 1u << 4,
    CRAN_TRANSPORT_SHARED_MEMORY = 1u << 5,
};

typedef std::uint64_t (*cran_monotonic_time_ns_fn)(void* context);
typedef int (*cran_write_channel_fn)(
    void* context,
    std::uint8_t channel,
    const std::uint8_t* data,
    std::size_t length);
typedef void (*cran_log_fn)(void* context, std::uint8_t level, const char* message);

struct cran_platform_ops {
    std::uint32_t abi_version;
    void* context;
    cran_monotonic_time_ns_fn monotonic_time_ns;
    cran_write_channel_fn write_channel;
    cran_log_fn log;
};

struct cran_target_capabilities {
    std::uint32_t schema_version;
    std::uint16_t word_size_bits;
    std::uint16_t float_width_bits;
    std::uint8_t little_endian;
    std::uint8_t reserved[3];
    std::uint32_t max_payload_bytes;
    std::uint32_t memory_bytes;
    std::uint32_t feature_mask;
    std::uint32_t transport_mask;
    std::uint8_t dynamic_allocation;
    std::uint8_t capability_reserved[3];
};

CRAN_API void* cran_target_create(
    std::uint32_t vehicle_id,
    double publish_rate_hz,
    const cran_platform_ops* platform,
    const cran_target_capabilities* capabilities);

CRAN_API void cran_target_destroy(void* runtime);
CRAN_API void cran_target_reset(void* runtime);

CRAN_API void cran_target_on_sensor(
    void* runtime,
    std::uint32_t sequence,
    std::uint64_t timestamp_ns,
    const double acceleration_mps2[3],
    const double angular_rate_rps[3],
    const double magnetic_field_ut[3],
    const double position_m[2],
    double heading_rad,
    double temperature_c,
    std::uint8_t flags);

CRAN_API void cran_target_on_input(
    void* runtime,
    std::uint8_t channel,
    const std::uint8_t* data,
    std::size_t length,
    std::uint64_t timestamp_ns);

// cran_target_poll uses the BSP clock. cran_target_poll_at is useful for an
// RTOS task or deterministic processor/HIL scheduler that already owns time.
CRAN_API int cran_target_poll(void* runtime);
CRAN_API int cran_target_poll_at(void* runtime, std::uint64_t timestamp_ns);

CRAN_API int cran_target_get_capabilities(
    void* runtime, cran_target_capabilities* destination);

}
