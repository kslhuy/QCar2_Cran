#include "cran_target_adapter.h"

#include <algorithm>
#include <new>
#include <vector>

namespace {

constexpr std::uint32_t kPlatformAbiVersion = 1;
constexpr std::size_t kInitialOutputCapacity = 65'536;

struct TargetRuntime {
    void* core = nullptr;
    cran_platform_ops platform{};
    cran_target_capabilities capabilities{};
    std::vector<std::uint8_t> output_buffer;
};

void log_message(TargetRuntime* runtime, std::uint8_t level, const char* message) {
    if (runtime != nullptr && runtime->platform.log != nullptr) {
        runtime->platform.log(runtime->platform.context, level, message);
    }
}

int drain_channel(TargetRuntime* runtime, std::uint8_t channel) {
    int written = 0;
    for (;;) {
        int length = cran_core_pop_output(
            runtime->core,
            channel,
            runtime->output_buffer.data(),
            runtime->output_buffer.size());
        if (length == 0) {
            return written;
        }
        if (length < 0) {
            const auto required = static_cast<std::size_t>(-length);
            if (required > runtime->capabilities.max_payload_bytes) {
                log_message(runtime, 3, "core output exceeds target payload capability");
                return -2;
            }
            runtime->output_buffer.resize(required);
            continue;
        }
        const int result = runtime->platform.write_channel(
            runtime->platform.context,
            channel,
            runtime->output_buffer.data(),
            static_cast<std::size_t>(length));
        if (result < 0) {
            log_message(runtime, 3, "platform write_channel rejected core output");
            return -3;
        }
        ++written;
    }
}

}  // namespace

extern "C" {

void* cran_target_create(
    std::uint32_t vehicle_id,
    double publish_rate_hz,
    const cran_platform_ops* platform,
    const cran_target_capabilities* capabilities) {
    if (platform == nullptr || capabilities == nullptr ||
        platform->abi_version != kPlatformAbiVersion ||
        platform->monotonic_time_ns == nullptr ||
        platform->write_channel == nullptr ||
        capabilities->max_payload_bytes == 0 ||
        capabilities->dynamic_allocation == 0 ||
        capabilities->memory_bytes < 65'536) {
        return nullptr;
    }
    auto* runtime = new (std::nothrow) TargetRuntime();
    if (runtime == nullptr) {
        return nullptr;
    }
    runtime->platform = *platform;
    runtime->capabilities = *capabilities;
    runtime->output_buffer.resize(
        std::min<std::size_t>(
            kInitialOutputCapacity, runtime->capabilities.max_payload_bytes));
    runtime->core = cran_core_create(vehicle_id, publish_rate_hz);
    if (runtime->core == nullptr) {
        delete runtime;
        return nullptr;
    }
    return runtime;
}

void cran_target_destroy(void* opaque) {
    auto* runtime = static_cast<TargetRuntime*>(opaque);
    if (runtime == nullptr) {
        return;
    }
    cran_core_destroy(runtime->core);
    delete runtime;
}

void cran_target_reset(void* opaque) {
    auto* runtime = static_cast<TargetRuntime*>(opaque);
    if (runtime != nullptr) {
        cran_core_reset(runtime->core);
    }
}

void cran_target_on_sensor(
    void* opaque,
    std::uint32_t sequence,
    std::uint64_t timestamp_ns,
    const double acceleration_mps2[3],
    const double angular_rate_rps[3],
    const double magnetic_field_ut[3],
    const double position_m[2],
    double heading_rad,
    double temperature_c,
    std::uint8_t flags) {
    auto* runtime = static_cast<TargetRuntime*>(opaque);
    if (runtime == nullptr) {
        return;
    }
    cran_core_on_nav(
        runtime->core,
        sequence,
        timestamp_ns,
        acceleration_mps2,
        angular_rate_rps,
        magnetic_field_ut,
        position_m,
        heading_rad,
        temperature_c,
        flags);
}

void cran_target_on_input(
    void* opaque,
    std::uint8_t channel,
    const std::uint8_t* data,
    std::size_t length,
    std::uint64_t timestamp_ns) {
    auto* runtime = static_cast<TargetRuntime*>(opaque);
    if (runtime != nullptr) {
        cran_core_on_input(runtime->core, channel, data, length, timestamp_ns);
    }
}

int cran_target_poll(void* opaque) {
    auto* runtime = static_cast<TargetRuntime*>(opaque);
    if (runtime == nullptr) {
        return -1;
    }
    return cran_target_poll_at(
        runtime,
        runtime->platform.monotonic_time_ns(runtime->platform.context));
}

int cran_target_poll_at(void* opaque, std::uint64_t timestamp_ns) {
    auto* runtime = static_cast<TargetRuntime*>(opaque);
    if (runtime == nullptr) {
        return -1;
    }
    cran_core_step(runtime->core, timestamp_ns);
    const int vehicle_count = drain_channel(runtime, CRAN_TARGET_VEHICLE_CHANNEL);
    if (vehicle_count < 0) {
        return vehicle_count;
    }
    const int v2v_count = drain_channel(runtime, CRAN_TARGET_V2V_CHANNEL);
    if (v2v_count < 0) {
        return v2v_count;
    }
    return vehicle_count + v2v_count;
}

int cran_target_get_capabilities(
    void* opaque, cran_target_capabilities* destination) {
    auto* runtime = static_cast<TargetRuntime*>(opaque);
    if (runtime == nullptr || destination == nullptr) {
        return -1;
    }
    *destination = runtime->capabilities;
    return 0;
}

}  // extern "C"
