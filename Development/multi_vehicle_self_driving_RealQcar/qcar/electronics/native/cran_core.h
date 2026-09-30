#pragma once

#include <cstddef>
#include <cstdint>

#if defined(CRAN_CORE_STATIC)
#  define CRAN_API
#elif defined(_WIN32)
#  if defined(CRAN_CORE_BUILD)
#    define CRAN_API __declspec(dllexport)
#  else
#    define CRAN_API __declspec(dllimport)
#  endif
#else
#  define CRAN_API __attribute__((visibility("default")))
#endif

extern "C" {

CRAN_API void* cran_core_create(std::uint32_t vehicle_id, double publish_rate_hz);
CRAN_API void cran_core_destroy(void* core);
CRAN_API void cran_core_reset(void* core);

CRAN_API void cran_core_on_nav(
    void* core,
    std::uint32_t sequence,
    std::uint64_t timestamp_ns,
    const double acceleration_mps2[3],
    const double angular_rate_rps[3],
    const double magnetic_field_ut[3],
    const double position_m[2],
    double heading_rad,
    double temperature_c,
    std::uint8_t flags);

// input_channel: 0 = vehicle, 1 = V2V
CRAN_API void cran_core_on_input(
    void* core,
    std::uint8_t input_channel,
    const std::uint8_t* data,
    std::size_t length,
    std::uint64_t timestamp_ns);

CRAN_API void cran_core_step(void* core, std::uint64_t timestamp_ns);

// output_channel: 0 = vehicle, 1 = V2V. Returns 0 when empty, a positive
// payload length on success, or the negative required capacity.
CRAN_API int cran_core_pop_output(
    void* core,
    std::uint8_t output_channel,
    std::uint8_t* destination,
    std::size_t capacity);

}
