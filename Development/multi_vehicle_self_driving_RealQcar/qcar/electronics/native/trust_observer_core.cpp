#include "trust_observer_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace {

constexpr double kEpsilon = 0.01;

double clip_unit(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, 0.0, 1.0);
}

double wrap_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct TargetTrustState {
    std::vector<double> trust_levels;
    std::vector<double> rating_local;
    std::vector<double> rating_global;
    std::vector<double> final_history;
};

struct TrustCore {
    CranTrustConfig config{};
    std::unordered_map<std::uint32_t, TargetTrustState> targets;
};

TargetTrustState make_target_state(std::uint32_t levels) {
    TargetTrustState state;
    state.trust_levels.assign(levels, 0.0);
    state.rating_local.assign(levels, 0.0);
    state.rating_global.assign(levels, 0.0);

    // Python's five-level TrustScore default is [0, 0, .2, .4, .4]. For a
    // non-default level count use a normalized neutral-to-positive prior.
    if (levels == 5U) {
        state.trust_levels = {0.0, 0.0, 0.2, 0.4, 0.4};
    } else {
        const std::uint32_t neutral = (levels - 1U) / 2U;
        state.trust_levels[neutral] = 0.2;
        const double remainder = 0.8 / static_cast<double>(levels - neutral - 1U);
        for (std::uint32_t i = neutral + 1U; i < levels; ++i) {
            state.trust_levels[i] = remainder;
        }
    }
    return state;
}

double matlab_trust_score(
    const std::vector<double>& rating,
    std::uint32_t levels,
    double regularization) {
    const double total = std::accumulate(rating.begin(), rating.end(), 0.0);
    const double denominator = regularization + total;
    if (denominator <= 0.0) {
        return 0.0;
    }
    double score = 0.0;
    for (std::uint32_t i = 0; i < levels; ++i) {
        const double probability =
            (rating[i] + regularization / static_cast<double>(levels)) / denominator;
        const double weight =
            (static_cast<double>(i) + kEpsilon) /
            (static_cast<double>(levels - 1U) + kEpsilon);
        score += weight * probability;
    }
    return score;
}

void matlab_update_rating(
    std::vector<double>& rating,
    double sample,
    std::uint32_t levels,
    double regularization,
    double aging_weight) {
    const double mapped = clip_unit(sample) * static_cast<double>(levels - 1U);
    // nearbyint follows round-to-nearest-even, matching numpy.round used by
    // the Python reference implementation under the default rounding mode.
    const auto level = static_cast<std::uint32_t>(std::clamp(
        std::nearbyint(mapped), 0.0, static_cast<double>(levels - 1U)));
    const double sigma = matlab_trust_score(rating, levels, regularization);
    const double lambda = sigma * aging_weight;
    for (double& value : rating) {
        value *= (1.0 - lambda);
    }
    rating[level] += 1.0;
}

void update_levels(
    TargetTrustState& state,
    const CranTrustConfig& config,
    double local_sample,
    double global_sample) {
    const double combined = config.dirichlet_dual != 0U
        ? clip_unit(local_sample) * clip_unit(global_sample)
        : clip_unit(local_sample);
    const auto level = static_cast<std::uint32_t>(std::clamp(
        static_cast<long long>(
            combined * static_cast<double>(config.num_trust_levels - 1U)),
        0LL,
        static_cast<long long>(config.num_trust_levels - 1U)));
    const double rate = clip_unit(config.dirichlet_update_rate);
    double total = 0.0;
    for (std::uint32_t i = 0; i < config.num_trust_levels; ++i) {
        state.trust_levels[i] =
            (1.0 - rate) * state.trust_levels[i] + (i == level ? rate : 0.0);
        total += state.trust_levels[i];
    }
    if (total > 0.0) {
        for (double& value : state.trust_levels) {
            value /= total;
        }
    }
}

double levels_final_score(const std::vector<double>& levels) {
    const double count = static_cast<double>(levels.size());
    double score = 0.0;
    for (std::size_t i = 0; i < levels.size(); ++i) {
        const double level_value = (static_cast<double>(i) + 0.5) / count;
        score += levels[i] * level_value;
    }
    return clip_unit(score);
}

double sudden_change_beta(
    const TargetTrustState& state,
    const CranTrustConfig& config,
    double global_sample) {
    const std::size_t window = std::max<std::size_t>(config.attack_detection_window, 2U);
    if (state.final_history.size() < window) {
        return 1.0;
    }
    const auto begin = state.final_history.end() - static_cast<std::ptrdiff_t>(window);
    const double mean = std::accumulate(begin, state.final_history.end(), 0.0) /
        static_cast<double>(window);
    double squared_sum = 0.0;
    for (auto it = begin; it != state.final_history.end(); ++it) {
        const double delta = *it - mean;
        squared_sum += delta * delta;
    }
    const double sigma = std::sqrt(squared_sum / static_cast<double>(window));
    if (sigma <= 1e-6) {
        return 1.0;
    }
    const double z = std::abs(global_sample - mean) / sigma;
    if (z <= std::max(config.sudden_change_threshold, 0.1)) {
        return 1.0;
    }
    return clip_unit(1.0 - 0.2 * std::min(z / 5.0, 1.0));
}

struct WeightCandidate {
    std::size_t original_index;
    std::uint32_t vehicle_id;
    double trust;
    double basis;
    double weight;
};

double interpolate_clamped(
    double value,
    const double* x_values,
    const double* y_values,
    std::size_t count) {
    if (count == 0U || x_values == nullptr || y_values == nullptr) {
        return 0.0;
    }
    if (value <= x_values[0]) {
        return y_values[0];
    }
    if (value >= x_values[count - 1U]) {
        return y_values[count - 1U];
    }
    for (std::size_t i = 1U; i < count; ++i) {
        if (value <= x_values[i]) {
            const double span = x_values[i] - x_values[i - 1U];
            if (std::abs(span) <= 1e-15) {
                return y_values[i];
            }
            const double alpha = (value - x_values[i - 1U]) / span;
            return y_values[i - 1U] + alpha * (y_values[i] - y_values[i - 1U]);
        }
    }
    return y_values[count - 1U];
}

void apply_state_constraints(
    double* state,
    std::size_t state_dim,
    double max_velocity,
    double max_acceleration) {
    if (state_dim > 2U) {
        state[2] = wrap_angle(state[2]);
    }
    if (state_dim > 3U) {
        state[3] = std::clamp(
            state[3], -std::abs(max_velocity), std::abs(max_velocity));
    }
    if (state_dim > 4U) {
        state[4] = std::clamp(
            state[4], -std::abs(max_acceleration), std::abs(max_acceleration));
    }
}

double blend_angles(
    double primary,
    double secondary,
    double primary_weight,
    double secondary_weight) {
    const double sin_sum =
        primary_weight * std::sin(primary) + secondary_weight * std::sin(secondary);
    const double cos_sum =
        primary_weight * std::cos(primary) + secondary_weight * std::cos(secondary);
    if (std::abs(sin_sum) <= 1e-9 && std::abs(cos_sum) <= 1e-9) {
        return primary;
    }
    return std::atan2(sin_sum, cos_sum);
}

void predict_vehicle_model(
    const double* state,
    std::size_t state_dim,
    const CranPredictionConfig& config,
    const double* throttle_breakpoints,
    const double* velocity_breakpoints,
    std::size_t lookup_count,
    double* output) {
    std::copy(state, state + state_dim, output);
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double velocity = state[3];
    const double acceleration = state_dim > 4U ? state[4] : 0.0;

    if (config.prediction_mode == CRAN_PREDICTION_DEAD_RECKONING) {
        output[0] = x + velocity * std::cos(theta) * config.dt;
        output[1] = y + velocity * std::sin(theta) * config.dt;
        return;
    }

    const bool has_control = config.has_control != 0U;
    const double steering = std::clamp(
        has_control ? config.steering : 0.0,
        -std::abs(config.max_steering),
        std::abs(config.max_steering));
    const double throttle = has_control ? config.throttle : 0.0;
    const double wheelbase = std::max(config.wheelbase, 1e-6);

    output[0] = x + velocity * std::cos(theta) * config.dt;
    output[1] = y + velocity * std::sin(theta) * config.dt;
    output[2] = theta + (velocity * std::tan(steering) / wheelbase) * config.dt;

    double velocity_new = velocity;
    double acceleration_new = 0.0;
    if (!has_control) {
        velocity_new = velocity;
    } else if (config.longitudinal_model == CRAN_LONGITUDINAL_VELOCITY_LAG) {
        double throttle_effective = throttle;
        const double deadband = std::max(config.velocity_lag_deadband, 0.0);
        if (deadband > 0.0) {
            const double sign = (throttle > 0.0) - (throttle < 0.0);
            throttle_effective =
                sign * std::max(std::abs(throttle) - deadband, 0.0);
        }
        const double tau = std::max(config.velocity_lag_tau, 1e-6);
        const double velocity_dot =
            -(1.0 / tau) * velocity + (config.velocity_gain / tau) * throttle_effective;
        velocity_new = velocity + velocity_dot * config.dt;
        acceleration_new = velocity_dot;
    } else if (
        config.longitudinal_model == CRAN_LONGITUDINAL_VELOCITY_LAG_LOOKUP) {
        double steady_velocity = 0.0;
        if (lookup_count >= 2U && throttle_breakpoints != nullptr &&
            velocity_breakpoints != nullptr) {
            steady_velocity = interpolate_clamped(
                throttle,
                throttle_breakpoints,
                velocity_breakpoints,
                lookup_count);
        } else {
            double throttle_effective = throttle;
            const double deadband = std::max(config.velocity_lag_deadband, 0.0);
            if (deadband > 0.0) {
                const double sign = (throttle > 0.0) - (throttle < 0.0);
                throttle_effective =
                    sign * std::max(std::abs(throttle) - deadband, 0.0);
            }
            steady_velocity = config.velocity_gain * throttle_effective;
        }
        const double velocity_dot =
            (steady_velocity - velocity) /
            std::max(config.velocity_lag_lookup_tau, 1e-6);
        velocity_new = velocity + velocity_dot * config.dt;
        acceleration_new = velocity_dot;
    } else if (
        config.longitudinal_model == CRAN_LONGITUDINAL_VELOCITY_COMMAND) {
        const double velocity_dot =
            (throttle - velocity) / std::max(config.velocity_command_tau, 1e-6);
        velocity_new = velocity + velocity_dot * config.dt;
        acceleration_new = velocity_dot;
    } else if (
        config.longitudinal_model == CRAN_LONGITUDINAL_ACCELERATION_LAG) {
        const double tau = std::max(config.accel_lag_tau, 1e-6);
        acceleration_new = acceleration + config.dt * (
            -(1.0 / tau) * acceleration +
            (config.accel_lag_gain / tau) * throttle);
        velocity_new = velocity + acceleration_new * config.dt;
    } else if (
        config.longitudinal_model == CRAN_LONGITUDINAL_SIMPLE_ACCELERATION) {
        acceleration_new = throttle;
        velocity_new = velocity + acceleration_new * config.dt;
    }
    output[3] = velocity_new;
    if (state_dim > 4U) {
        output[4] = acceleration_new;
    }
}

}  // namespace

extern "C" {

void* cran_trust_create(const CranTrustConfig* config) {
    if (config == nullptr || config->num_trust_levels < 2U ||
        config->num_trust_levels > CRAN_TRUST_MAX_LEVELS ||
        config->dirichlet_c <= 0.0) {
        return nullptr;
    }
    auto* core = new TrustCore();
    core->config = *config;
    return core;
}

void cran_trust_destroy(void* core) {
    delete static_cast<TrustCore*>(core);
}

void cran_trust_reset(void* core) {
    if (core != nullptr) {
        static_cast<TrustCore*>(core)->targets.clear();
    }
}

double cran_trust_local_sample(const double scores[6]) {
    if (scores == nullptr) {
        return 0.0;
    }
    constexpr std::array<double, 6> weights = {0.3, 0.2, 0.15, 0.15, 0.1, 0.1};
    double log_sum = 0.0;
    double total_weight = 0.0;
    for (std::size_t i = 0; i < weights.size(); ++i) {
        const double safe_score = std::max(std::isfinite(scores[i]) ? scores[i] : 0.0, 0.01);
        log_sum += weights[i] * std::log(safe_score);
        total_weight += weights[i];
    }
    return clip_unit(std::exp(log_sum / total_weight));
}

int cran_trust_step(
    void* core_ptr,
    std::uint32_t target_id,
    double local_trust_sample,
    double global_trust_sample,
    std::uint8_t missing_observation,
    CranTrustResult* result) {
    if (core_ptr == nullptr || result == nullptr) {
        return -1;
    }
    auto& core = *static_cast<TrustCore*>(core_ptr);
    auto insertion = core.targets.emplace(
        target_id, make_target_state(core.config.num_trust_levels));
    auto& state = insertion.first->second;
    local_trust_sample = clip_unit(local_trust_sample);
    global_trust_sample = clip_unit(global_trust_sample);

    update_levels(state, core.config, local_trust_sample, global_trust_sample);

    double final_score = 0.0;
    if (missing_observation == 0U &&
        core.config.dirichlet_method == CRAN_DIRICHLET_MATLAB) {
        if (core.config.dirichlet_dual == 0U) {
            matlab_update_rating(
                state.rating_local,
                local_trust_sample * global_trust_sample,
                core.config.num_trust_levels,
                core.config.dirichlet_c,
                core.config.dirichlet_wt_local);
            final_score = matlab_trust_score(
                state.rating_local,
                core.config.num_trust_levels,
                core.config.dirichlet_c);
        } else {
            matlab_update_rating(
                state.rating_local,
                local_trust_sample,
                core.config.num_trust_levels,
                core.config.dirichlet_c,
                core.config.dirichlet_wt_local);
            matlab_update_rating(
                state.rating_global,
                global_trust_sample,
                core.config.num_trust_levels,
                core.config.dirichlet_c,
                core.config.dirichlet_wt_global);
            final_score =
                matlab_trust_score(
                    state.rating_local,
                    core.config.num_trust_levels,
                    core.config.dirichlet_c) *
                matlab_trust_score(
                    state.rating_global,
                    core.config.num_trust_levels,
                    core.config.dirichlet_c);
        }
    } else {
        final_score = levels_final_score(state.trust_levels);
    }

    if (missing_observation == 0U && core.config.monitor_sudden_change != 0U) {
        final_score *= sudden_change_beta(state, core.config, global_trust_sample);
    }
    if (!state.final_history.empty()) {
        const double alpha = clip_unit(core.config.ema_alpha);
        final_score = alpha * final_score + (1.0 - alpha) * state.final_history.back();
    }
    final_score = clip_unit(final_score);
    state.final_history.push_back(final_score);
    const std::size_t history_limit = std::max<std::size_t>(
        50U, static_cast<std::size_t>(core.config.attack_detection_window));
    if (state.final_history.size() > history_limit) {
        state.final_history.erase(state.final_history.begin());
    }

    result->final_score = final_score;
    result->trust_level_count = core.config.num_trust_levels;
    std::fill(
        std::begin(result->trust_levels), std::end(result->trust_levels), 0.0);
    std::copy(
        state.trust_levels.begin(),
        state.trust_levels.end(),
        result->trust_levels);
    const double threshold = clip_unit(core.config.trust_threshold);
    result->flag_target_attack = static_cast<std::uint8_t>(
        local_trust_sample < threshold && global_trust_sample < threshold);
    result->flag_global_est_check = static_cast<std::uint8_t>(
        local_trust_sample >= threshold && global_trust_sample < threshold);
    result->flag_local_est_check = static_cast<std::uint8_t>(
        local_trust_sample < threshold);
    result->reserved = 0U;
    return 0;
}

int cran_observer_correct(
    const double* current_state,
    std::size_t state_dim,
    const double* direct_state,
    double direct_weight,
    const double* neighbor_states,
    const double* neighbor_weights,
    std::size_t neighbor_count,
    double max_velocity,
    double max_acceleration,
    double* corrected_state) {
    if (current_state == nullptr || corrected_state == nullptr || state_dim == 0U ||
        (neighbor_count > 0U &&
         (neighbor_states == nullptr || neighbor_weights == nullptr))) {
        return -1;
    }
    for (std::size_t axis = 0; axis < state_dim; ++axis) {
        corrected_state[axis] = current_state[axis];
    }

    const auto add_residual = [&](const double* measurement, double weight) {
        if (measurement == nullptr || weight <= 0.0) {
            return;
        }
        for (std::size_t axis = 0; axis < state_dim; ++axis) {
            double residual = measurement[axis] - current_state[axis];
            if (axis == 2U) {
                residual = wrap_angle(residual);
            }
            corrected_state[axis] += weight * residual;
        }
    };

    add_residual(direct_state, direct_weight);
    for (std::size_t neighbor = 0; neighbor < neighbor_count; ++neighbor) {
        add_residual(
            neighbor_states + neighbor * state_dim,
            neighbor_weights[neighbor]);
    }
    if (state_dim > 2U) {
        corrected_state[2] = wrap_angle(corrected_state[2]);
    }
    if (state_dim > 3U) {
        corrected_state[3] = std::clamp(
            corrected_state[3], -std::abs(max_velocity), std::abs(max_velocity));
    }
    if (state_dim > 4U) {
        corrected_state[4] = std::clamp(
            corrected_state[4], -std::abs(max_acceleration), std::abs(max_acceleration));
    }
    return 0;
}

int cran_observer_weights(
    const CranWeightConfig* config,
    const CranWeightInput* input,
    const std::uint32_t* neighbor_ids,
    const double* neighbor_trust_scores,
    double* neighbor_weights,
    CranWeightResult* result) {
    if (config == nullptr || input == nullptr || result == nullptr ||
        (input->neighbor_count > 0U &&
         (neighbor_ids == nullptr || neighbor_trust_scores == nullptr ||
          neighbor_weights == nullptr))) {
        return -1;
    }
    const std::size_t count = input->neighbor_count;
    for (std::size_t i = 0; i < count; ++i) {
        neighbor_weights[i] = 0.0;
    }
    std::vector<WeightCandidate> candidates;
    candidates.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        candidates.push_back(WeightCandidate{
            i,
            neighbor_ids[i],
            std::isfinite(neighbor_trust_scores[i])
                ? neighbor_trust_scores[i]
                : 0.0,
            0.0,
            0.0});
    }

    double direct_weight = 0.0;
    double self_weight = 1.0;
    std::vector<WeightCandidate*> selected;

    const auto cap_neighbors = [&]() {
        if (selected.empty()) {
            return 0.0;
        }
        double overflow_to_self = 0.0;
        constexpr double tolerance = 1e-12;
        while (true) {
            std::vector<WeightCandidate*> over_cap;
            for (auto* candidate : selected) {
                if (candidate->weight > config->w_cap + tolerance) {
                    over_cap.push_back(candidate);
                }
            }
            if (over_cap.empty()) {
                break;
            }
            double overflow = 0.0;
            for (auto* candidate : over_cap) {
                overflow += candidate->weight - config->w_cap;
                candidate->weight = config->w_cap;
            }
            std::vector<WeightCandidate*> uncapped;
            for (auto* candidate : selected) {
                if (candidate->weight < config->w_cap - tolerance) {
                    uncapped.push_back(candidate);
                }
            }
            if (overflow <= tolerance || uncapped.empty()) {
                overflow_to_self += overflow;
                break;
            }
            double basis_sum = 0.0;
            for (const auto* candidate : uncapped) {
                basis_sum += std::max(candidate->basis, 0.0);
            }
            if (basis_sum <= tolerance) {
                overflow_to_self += overflow;
                break;
            }
            for (auto* candidate : uncapped) {
                candidate->weight +=
                    overflow * (std::max(candidate->basis, 0.0) / basis_sum);
            }
        }
        return overflow_to_self;
    };

    if (input->mode == CRAN_WEIGHT_STARTUP) {
        const std::size_t limit = std::min<std::size_t>(config->kappa, count);
        for (std::size_t i = 0; i < limit; ++i) {
            selected.push_back(&candidates[i]);
        }
        direct_weight = input->direct_available != 0U
            ? std::max(config->w0_fixed, 0.0)
            : 0.0;
        self_weight = std::max(config->w_self_base, 0.0);
        const double neighbor_budget =
            std::max(0.0, 1.0 - direct_weight - self_weight);
        if (!selected.empty() && neighbor_budget > 0.0) {
            const double basis = 1.0 / static_cast<double>(selected.size());
            for (auto* candidate : selected) {
                candidate->basis = basis;
                candidate->weight = neighbor_budget * basis;
            }
            self_weight += cap_neighbors();
        } else {
            self_weight += neighbor_budget;
        }
    } else if (input->mode == CRAN_WEIGHT_EQUAL) {
        for (auto& candidate : candidates) {
            if (candidate.trust >= config->trust_threshold &&
                selected.size() < config->kappa) {
                selected.push_back(&candidate);
            }
        }
        const std::size_t channel_count = selected.size() +
            (input->direct_available != 0U ? 1U : 0U);
        if (channel_count > 0U) {
            const double equal_weight = 1.0 / static_cast<double>(channel_count);
            direct_weight = input->direct_available != 0U ? equal_weight : 0.0;
            for (auto* candidate : selected) {
                candidate->weight = equal_weight;
            }
            self_weight = 0.0;
        }
    } else if (input->mode == CRAN_WEIGHT_PAPER) {
        for (auto& candidate : candidates) {
            if (candidate.trust >= config->trust_threshold) {
                selected.push_back(&candidate);
            }
        }
        const bool include_anchor =
            input->direct_available != 0U &&
            input->target_local_trust >= config->trust_threshold;
        const std::size_t legitimate_count =
            selected.size() + (include_anchor ? 1U : 0U);
        const std::size_t denominator = std::max<std::size_t>(
            config->kappa, legitimate_count + 1U);
        const double base_weight = 1.0 / static_cast<double>(denominator);
        direct_weight = include_anchor ? base_weight : 0.0;
        for (auto* candidate : selected) {
            candidate->weight = base_weight;
        }
        self_weight = std::max(
            0.0,
            1.0 - direct_weight -
                base_weight * static_cast<double>(selected.size()));
    } else {
        for (auto& candidate : candidates) {
            if (candidate.trust >= config->trust_threshold) {
                selected.push_back(&candidate);
            }
        }
        std::stable_sort(
            selected.begin(),
            selected.end(),
            [](const WeightCandidate* left, const WeightCandidate* right) {
                return left->trust > right->trust;
            });
        if (selected.size() > config->kappa) {
            selected.resize(config->kappa);
        }

        double direct_factor = 1.0;
        double self_factor = 1.0;
        double neighbor_factor = 1.0;
        if (input->flag_target_attack != 0U) {
            direct_factor = std::min(
                1.0, std::max(0.0, config->flag_w0_target_attack_factor));
            neighbor_factor = direct_factor;
            self_factor = 1.0 + (1.0 - direct_factor);
        } else if (input->flag_global_est_check != 0U) {
            direct_factor = std::max(1.0, config->flag_w0_global_est_check_factor);
            neighbor_factor = std::max(0.0, 1.0 / direct_factor);
        } else if (input->flag_local_est_check != 0U) {
            direct_factor = std::min(
                1.0, std::max(0.0, config->flag_w0_local_est_check_factor));
            neighbor_factor = 1.0 + (1.0 - direct_factor);
        }

        if (config->use_gamma_self_weight_adaptation != 0U &&
            input->has_target_trust != 0U) {
            const double floor = clip_unit(config->gamma_self_weight_floor);
            self_factor *= floor + (1.0 - floor) * clip_unit(input->gamma_self);
        }
        const double anchor_gain = std::max(config->w0_fixed, 0.0);
        const double self_gain = std::max(config->w_self_base, 0.0);
        const double neighbor_gain =
            std::max(0.0, 1.0 - anchor_gain - self_gain);
        const double direct_reliability = input->direct_available != 0U
            ? clip_unit(input->target_local_trust)
            : 0.0;
        const double direct_raw =
            anchor_gain * direct_reliability * direct_factor;
        const double self_raw = self_gain * self_factor;
        double trust_sum = 0.0;
        for (const auto* candidate : selected) {
            trust_sum += std::max(candidate->trust, 0.0);
        }
        double neighbor_raw_sum = 0.0;
        if (trust_sum > 0.0) {
            for (auto* candidate : selected) {
                candidate->basis = std::max(candidate->trust, 0.0) / trust_sum;
                candidate->weight =
                    neighbor_gain * neighbor_factor * candidate->basis;
                neighbor_raw_sum += candidate->weight;
            }
        }
        const double raw_total = direct_raw + self_raw + neighbor_raw_sum;
        if (raw_total > 0.0) {
            direct_weight = direct_raw / raw_total;
            self_weight = self_raw / raw_total;
            for (auto* candidate : selected) {
                candidate->weight /= raw_total;
            }
            self_weight += cap_neighbors();
        }

        if (input->flag_local_est_check != 0U && direct_weight <= 1e-9) {
            double total_neighbor = 0.0;
            for (const auto* candidate : selected) {
                total_neighbor += std::max(candidate->weight, 0.0);
            }
            const double cap_total =
                clip_unit(config->local_bad_zero_w0_neighbor_total_cap);
            if (total_neighbor > cap_total + 1e-12) {
                const double scale = cap_total / std::max(total_neighbor, 1e-12);
                for (auto* candidate : selected) {
                    candidate->weight = std::max(candidate->weight, 0.0) * scale;
                }
                self_weight = 1.0 - direct_weight - cap_total;
            } else if (selected.empty()) {
                self_weight = 1.0 - direct_weight;
            }
        }
    }

    double used = direct_weight + self_weight;
    for (const auto* candidate : selected) {
        used += candidate->weight;
    }
    if (std::abs(1.0 - used) > 1e-12) {
        self_weight = std::max(0.0, self_weight + (1.0 - used));
    }

    const double recovery_scale = clip_unit(input->direct_recovery_scale);
    if (recovery_scale < 1.0 - 1e-12) {
        const double old_direct = std::max(direct_weight, 0.0);
        direct_weight = old_direct * recovery_scale;
        self_weight = std::max(self_weight, 0.0) + old_direct - direct_weight;
    }

    double neighbor_total = 0.0;
    for (const auto* candidate : selected) {
        neighbor_weights[candidate->original_index] = candidate->weight;
        neighbor_total += candidate->weight;
    }
    used = direct_weight + self_weight + neighbor_total;
    if (std::abs(1.0 - used) > 1e-12) {
        self_weight = std::max(0.0, self_weight + (1.0 - used));
    }
    result->direct_weight = direct_weight;
    result->self_weight = self_weight;
    result->total_neighbor_weight = neighbor_total;
    return 0;
}

int cran_observer_predict(
    const double* corrected_state,
    std::size_t state_dim,
    const double* clean_state,
    const CranPredictionConfig* config,
    const CranHostAnchor* anchor,
    const double* throttle_breakpoints,
    const double* velocity_breakpoints,
    std::size_t lookup_count,
    double* predicted_state) {
    if (corrected_state == nullptr || config == nullptr || predicted_state == nullptr ||
        state_dim < 4U ||
        (config->has_clean_state != 0U && clean_state == nullptr) ||
        (config->has_anchor != 0U && anchor == nullptr) ||
        (lookup_count > 0U &&
         (throttle_breakpoints == nullptr || velocity_breakpoints == nullptr))) {
        return -1;
    }
    std::copy(corrected_state, corrected_state + state_dim, predicted_state);
    if (config->dt <= 0.0 || config->prediction_mode == CRAN_PREDICTION_NONE) {
        apply_state_constraints(
            predicted_state, state_dim, config->max_velocity, config->max_acceleration);
        return 0;
    }

    if (config->prediction_mode == CRAN_PREDICTION_CLEAN_DATA) {
        const bool has_clean = config->has_clean_state != 0U;
        const double theta = has_clean ? clean_state[2] : corrected_state[2];
        const double velocity = has_clean ? clean_state[3] : corrected_state[3];
        const double acceleration = has_clean && state_dim > 4U
            ? clean_state[4]
            : (state_dim > 4U ? corrected_state[4] : 0.0);
        double base_x = corrected_state[0];
        double base_y = corrected_state[1];
        if (config->force_clean_pose_anchor != 0U && has_clean) {
            base_x = clean_state[0];
            base_y = clean_state[1];
        }
        predicted_state[0] = base_x + velocity * std::cos(theta) * config->dt;
        predicted_state[1] = base_y + velocity * std::sin(theta) * config->dt;
        predicted_state[2] = wrap_angle(theta);
        predicted_state[3] = velocity;
        if (state_dim > 4U) {
            predicted_state[4] = acceleration;
        }
    } else if (
        config->prediction_mode == CRAN_PREDICTION_RELATIVE_HOST_ANCHOR_MIXED &&
        config->attack_anchor_active != 0U) {
        std::vector<double> model_predicted(state_dim, 0.0);
        CranPredictionConfig model_config = *config;
        model_config.prediction_mode = CRAN_PREDICTION_MODEL;
        predict_vehicle_model(
            corrected_state,
            state_dim,
            model_config,
            throttle_breakpoints,
            velocity_breakpoints,
            lookup_count,
            model_predicted.data());

        const bool has_clean = config->has_clean_state != 0U;
        const bool has_anchor = config->has_anchor != 0U;
        double theta = corrected_state[2];
        double velocity = corrected_state[3];
        double acceleration = state_dim > 4U ? corrected_state[4] : 0.0;
        const double host_theta = has_anchor ? anchor->host_theta : theta;
        theta = has_clean
            ? blend_angles(
                  clean_state[2],
                  host_theta,
                  config->clean_theta_weight,
                  config->host_theta_weight)
            : host_theta;
        if (has_anchor) {
            velocity =
                config->target_velocity_weight * velocity +
                config->host_velocity_weight * anchor->host_velocity;
            acceleration =
                config->target_acceleration_weight * acceleration +
                config->host_acceleration_weight * anchor->host_acceleration;
        }

        double base_x = corrected_state[0];
        double base_y = corrected_state[1];
        bool has_position_anchor = false;
        if (has_anchor) {
            const double distance = std::max(anchor->distance, 0.1);
            const double sign = anchor->sign >= 0.0 ? 1.0 : -1.0;
            if (config->use_anchor_bearing != 0U &&
                std::isfinite(anchor->relative_x) &&
                std::isfinite(anchor->relative_y)) {
                const double cosine = std::cos(host_theta);
                const double sine = std::sin(host_theta);
                base_x =
                    anchor->host_x + cosine * anchor->relative_x - sine * anchor->relative_y;
                base_y =
                    anchor->host_y + sine * anchor->relative_x + cosine * anchor->relative_y;
            } else {
                base_x = anchor->host_x + sign * distance * std::cos(host_theta);
                base_y = anchor->host_y + sign * distance * std::sin(host_theta);
            }
            has_position_anchor = true;
        } else if (config->force_clean_pose_anchor != 0U && has_clean) {
            base_x = clean_state[0];
            base_y = clean_state[1];
            has_position_anchor = true;
        }

        const double anchor_x = base_x + velocity * std::cos(theta) * config->dt;
        const double anchor_y = base_y + velocity * std::sin(theta) * config->dt;
        if (has_position_anchor) {
            const double anchor_weight = std::max(config->anchor_position_weight, 0.0);
            const double estimate_weight = std::max(config->estimate_position_weight, 0.0);
            const double total_weight = anchor_weight + estimate_weight;
            if (total_weight > 1e-9) {
                predicted_state[0] =
                    (anchor_weight * anchor_x + estimate_weight * model_predicted[0]) /
                    total_weight;
                predicted_state[1] =
                    (anchor_weight * anchor_y + estimate_weight * model_predicted[1]) /
                    total_weight;
            } else {
                predicted_state[0] = anchor_x;
                predicted_state[1] = anchor_y;
            }
        } else {
            predicted_state[0] = anchor_x;
            predicted_state[1] = anchor_y;
        }
        predicted_state[2] = wrap_angle(theta);
        predicted_state[3] = velocity;
        if (state_dim > 4U) {
            predicted_state[4] = acceleration;
        }
    } else {
        predict_vehicle_model(
            corrected_state,
            state_dim,
            *config,
            throttle_breakpoints,
            velocity_breakpoints,
            lookup_count,
            predicted_state);
    }

    apply_state_constraints(
        predicted_state, state_dim, config->max_velocity, config->max_acceleration);
    return 0;
}

}  // extern "C"
