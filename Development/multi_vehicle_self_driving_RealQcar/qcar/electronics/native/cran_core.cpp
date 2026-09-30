#include "cran_core.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace {

struct SensorFrame {
    std::uint32_t sequence{};
    std::uint64_t timestamp_ns{};
    double acceleration[3]{};
    double angular_rate[3]{};
    double magnetic_field[3]{};
    double position[2]{};
    double heading{};
    double temperature{};
    std::uint8_t flags{};
};

struct CranCore {
    std::uint32_t vehicle_id{};
    std::uint64_t publish_period_ns{};
    std::uint64_t last_publish_ns{std::numeric_limits<std::uint64_t>::max()};
    bool has_nav_frame{false};
    SensorFrame nav{};
    std::deque<std::vector<std::uint8_t>> vehicle_output;
    std::deque<std::vector<std::uint8_t>> v2v_output;
};

std::vector<std::uint8_t> make_sensor_json(const CranCore& core, std::uint64_t now_ns) {
    const auto& s = core.nav;
    char text[1400];
    const int length = std::snprintf(
        text,
        sizeof(text),
        "{\"type\":\"electronics_sensor\",\"vehicle_id\":%u,\"timestamp_ns\":%llu,"
        "\"sensor\":{\"sequence\":%u,\"timestamp_ns\":%llu,"
        "\"acceleration_mps2\":[%.17g,%.17g,%.17g],"
        "\"angular_rate_rps\":[%.17g,%.17g,%.17g],"
        "\"magnetic_field_ut\":[%.17g,%.17g,%.17g],"
        "\"position_m\":[%.17g,%.17g],\"heading_rad\":%.17g,"
        "\"temperature_c\":%.17g,\"imu_valid\":%s,"
        "\"magnetometer_valid\":%s,\"gnss_valid\":%s,\"gnss_fresh\":%s}}",
        static_cast<unsigned>(core.vehicle_id),
        static_cast<unsigned long long>(now_ns),
        static_cast<unsigned>(s.sequence),
        static_cast<unsigned long long>(s.timestamp_ns),
        s.acceleration[0], s.acceleration[1], s.acceleration[2],
        s.angular_rate[0], s.angular_rate[1], s.angular_rate[2],
        s.magnetic_field[0], s.magnetic_field[1], s.magnetic_field[2],
        s.position[0], s.position[1], s.heading, s.temperature,
        (s.flags & 0x01U) ? "true" : "false",
        (s.flags & 0x02U) ? "true" : "false",
        (s.flags & 0x04U) ? "true" : "false",
        (s.flags & 0x08U) ? "true" : "false");
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(text)) {
        return {};
    }
    return std::vector<std::uint8_t>(text, text + length);
}

}  // namespace

extern "C" {

void* cran_core_create(std::uint32_t vehicle_id, double publish_rate_hz) {
    auto* core = new CranCore{};
    core->vehicle_id = vehicle_id;
    if (publish_rate_hz > 0.0) {
        core->publish_period_ns = static_cast<std::uint64_t>(1.0e9 / publish_rate_hz);
        core->publish_period_ns = std::max<std::uint64_t>(1U, core->publish_period_ns);
    }
    return core;
}

void cran_core_destroy(void* core) {
    delete static_cast<CranCore*>(core);
}

void cran_core_reset(void* opaque) {
    auto* core = static_cast<CranCore*>(opaque);
    if (!core) return;
    core->last_publish_ns = std::numeric_limits<std::uint64_t>::max();
    core->has_nav_frame = false;
    core->nav = {};
    core->vehicle_output.clear();
    core->v2v_output.clear();
}

void cran_core_on_nav(
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
    auto* core = static_cast<CranCore*>(opaque);
    if (!core || !acceleration_mps2 || !angular_rate_rps || !magnetic_field_ut || !position_m) return;
    core->nav.sequence = sequence;
    core->nav.timestamp_ns = timestamp_ns;
    std::copy_n(acceleration_mps2, 3, core->nav.acceleration);
    std::copy_n(angular_rate_rps, 3, core->nav.angular_rate);
    std::copy_n(magnetic_field_ut, 3, core->nav.magnetic_field);
    std::copy_n(position_m, 2, core->nav.position);
    core->nav.heading = heading_rad;
    core->nav.temperature = temperature_c;
    core->nav.flags = flags;
    core->has_nav_frame = true;
}

void cran_core_on_input(
    void* opaque,
    std::uint8_t input_channel,
    const std::uint8_t* data,
    std::size_t length,
    std::uint64_t timestamp_ns) {
    auto* core = static_cast<CranCore*>(opaque);
    if (!core || (!data && length > 0U)) return;
    const bool host_v2v_tx =
        input_channel == 0U && length >= 4U &&
        data[0] == 'E' && data[1] == 'V' && data[2] == 'T' && data[3] == '1';
    const bool host_v2v_mirror =
        input_channel == 0U && length >= 4U &&
        data[0] == 'E' && data[1] == 'V' && data[2] == 'M' && data[3] == '1';
    if (host_v2v_tx) {
        core->v2v_output.emplace_back(data, data + length);
    } else if (input_channel == 1U) {
        std::vector<std::uint8_t> envelope{'E', 'V', 'R', '1'};
        envelope.insert(envelope.end(), data, data + length);
        core->vehicle_output.push_back(std::move(envelope));
    } else if (host_v2v_mirror) {
        // Observation-only path; deliberately no radio output.
    }
    (void)timestamp_ns;
}

void cran_core_step(void* opaque, std::uint64_t timestamp_ns) {
    auto* core = static_cast<CranCore*>(opaque);
    if (!core || !core->has_nav_frame || core->publish_period_ns == 0U) return;
    const bool first = core->last_publish_ns == std::numeric_limits<std::uint64_t>::max();
    if (!first && timestamp_ns - core->last_publish_ns < core->publish_period_ns) return;
    auto payload = make_sensor_json(*core, timestamp_ns);
    if (!payload.empty()) core->vehicle_output.push_back(std::move(payload));
    core->last_publish_ns = timestamp_ns;
}

int cran_core_pop_output(
    void* opaque,
    std::uint8_t output_channel,
    std::uint8_t* destination,
    std::size_t capacity) {
    auto* core = static_cast<CranCore*>(opaque);
    if (!core) return 0;
    auto& queue = output_channel == 1U ? core->v2v_output : core->vehicle_output;
    if (queue.empty()) return 0;
    const auto& payload = queue.front();
    if (capacity < payload.size()) return -static_cast<int>(payload.size());
    if (!payload.empty() && destination) std::memcpy(destination, payload.data(), payload.size());
    const int length = static_cast<int>(payload.size());
    queue.pop_front();
    return length;
}

}  // extern "C"
