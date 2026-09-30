#pragma once

#include "cran_core.h"

#include <cstddef>
#include <cstdint>

// The ABI intentionally contains only fixed-width scalar values and flat
// arrays.  It can therefore be called from ctypes in SIL and from an STM32 HAL
// adapter without bringing Python, sockets, or dynamic message formats into
// the algorithm core.

constexpr std::uint32_t CRAN_TRUST_MAX_LEVELS = 8U;

enum CranDirichletMethod : std::uint8_t {
    CRAN_DIRICHLET_EMA = 0U,
    CRAN_DIRICHLET_MATLAB = 1U,
};

enum CranWeightMode : std::uint8_t {
    CRAN_WEIGHT_STARTUP = 0U,
    CRAN_WEIGHT_EQUAL = 1U,
    CRAN_WEIGHT_TRUST_BASED = 2U,
    CRAN_WEIGHT_PAPER = 3U,
};

enum CranPredictionMode : std::uint8_t {
    CRAN_PREDICTION_NONE = 0U,
    CRAN_PREDICTION_MODEL = 1U,
    CRAN_PREDICTION_DEAD_RECKONING = 2U,
    CRAN_PREDICTION_CLEAN_DATA = 3U,
    CRAN_PREDICTION_RELATIVE_HOST_ANCHOR_MIXED = 4U,
};

enum CranLongitudinalModel : std::uint8_t {
    CRAN_LONGITUDINAL_CONSTANT_VELOCITY = 0U,
    CRAN_LONGITUDINAL_VELOCITY_LAG = 1U,
    CRAN_LONGITUDINAL_VELOCITY_LAG_LOOKUP = 2U,
    CRAN_LONGITUDINAL_VELOCITY_COMMAND = 3U,
    CRAN_LONGITUDINAL_ACCELERATION_LAG = 4U,
    CRAN_LONGITUDINAL_SIMPLE_ACCELERATION = 5U,
};

struct CranTrustConfig {
    std::uint32_t num_trust_levels;
    double dirichlet_update_rate;
    double dirichlet_c;
    double dirichlet_wt_local;
    double dirichlet_wt_global;
    double ema_alpha;
    double trust_threshold;
    double sudden_change_threshold;
    std::uint32_t attack_detection_window;
    std::uint8_t dirichlet_method;
    std::uint8_t dirichlet_dual;
    std::uint8_t monitor_sudden_change;
    std::uint8_t reserved;
};

struct CranTrustResult {
    double final_score;
    double trust_levels[CRAN_TRUST_MAX_LEVELS];
    std::uint32_t trust_level_count;
    std::uint8_t flag_target_attack;
    std::uint8_t flag_global_est_check;
    std::uint8_t flag_local_est_check;
    std::uint8_t reserved;
};

struct CranWeightConfig {
    double w0_fixed;
    double w_self_base;
    double w_cap;
    double trust_threshold;
    double gamma_self_weight_floor;
    double flag_w0_target_attack_factor;
    double flag_w0_global_est_check_factor;
    double flag_w0_local_est_check_factor;
    double local_bad_zero_w0_neighbor_total_cap;
    std::uint32_t kappa;
    std::uint8_t use_gamma_self_weight_adaptation;
    std::uint8_t reserved[3];
};

struct CranWeightInput {
    double target_local_trust;
    double gamma_self;
    double direct_recovery_scale;
    std::uint32_t neighbor_count;
    std::uint8_t mode;
    std::uint8_t direct_available;
    std::uint8_t flag_target_attack;
    std::uint8_t flag_global_est_check;
    std::uint8_t flag_local_est_check;
    std::uint8_t has_target_trust;
    std::uint8_t reserved[2];
};

struct CranWeightResult {
    double direct_weight;
    double self_weight;
    double total_neighbor_weight;
};

struct CranPredictionConfig {
    double dt;
    double steering;
    double throttle;
    double wheelbase;
    double max_velocity;
    double max_acceleration;
    double max_steering;
    double velocity_lag_tau;
    double velocity_gain;
    double velocity_lag_deadband;
    double velocity_lag_lookup_tau;
    double velocity_command_tau;
    double accel_lag_tau;
    double accel_lag_gain;
    double anchor_position_weight;
    double estimate_position_weight;
    double clean_theta_weight;
    double host_theta_weight;
    double target_velocity_weight;
    double host_velocity_weight;
    double target_acceleration_weight;
    double host_acceleration_weight;
    std::uint8_t prediction_mode;
    std::uint8_t longitudinal_model;
    std::uint8_t has_control;
    std::uint8_t force_clean_pose_anchor;
    std::uint8_t attack_anchor_active;
    std::uint8_t has_clean_state;
    std::uint8_t has_anchor;
    std::uint8_t use_anchor_bearing;
};

struct CranHostAnchor {
    double host_x;
    double host_y;
    double host_theta;
    double host_velocity;
    double host_acceleration;
    double distance;
    double sign;
    double relative_x;
    double relative_y;
};

extern "C" {

CRAN_API void* cran_trust_create(const CranTrustConfig* config);
CRAN_API void cran_trust_destroy(void* core);
CRAN_API void cran_trust_reset(void* core);

// scores order: velocity, distance, acceleration, heading, beacon, quality.
CRAN_API double cran_trust_local_sample(const double scores[6]);

// local/global samples have already passed the sensor-specific consistency
// gates. missing_observation reproduces the Python packet-loss update, which
// updates trust levels but intentionally leaves MATLAB rating vectors intact.
CRAN_API int cran_trust_step(
    void* core,
    std::uint32_t target_id,
    double local_trust_sample,
    double global_trust_sample,
    std::uint8_t missing_observation,
    CranTrustResult* result);

// Trust-weighted distributed-observer correction:
//   x+ = x + w0(z0-x) + sum_l wl(zl-x)
// Heading residuals and the corrected heading use circular wrapping. Velocity
// and acceleration are clamped exactly like the current Python estimator.
CRAN_API int cran_observer_correct(
    const double* current_state,
    std::size_t state_dim,
    const double* direct_state,
    double direct_weight,
    const double* neighbor_states,
    const double* neighbor_weights,
    std::size_t neighbor_count,
    double max_velocity,
    double max_acceleration,
    double* corrected_state);

// Candidate neighbors are pre-filtered only for topology/availability. The
// kernel applies trust thresholds, kappa, flag adaptation, influence caps and
// direct recovery. Output weights use the same order as neighbor_ids/scores.
CRAN_API int cran_observer_weights(
    const CranWeightConfig* config,
    const CranWeightInput* input,
    const std::uint32_t* neighbor_ids,
    const double* neighbor_trust_scores,
    double* neighbor_weights,
    CranWeightResult* result);

// Pure prediction kernel used after consensus correction. Lookup arrays are
// optional and only used by CRAN_LONGITUDINAL_VELOCITY_LAG_LOOKUP.
CRAN_API int cran_observer_predict(
    const double* corrected_state,
    std::size_t state_dim,
    const double* clean_state,
    const CranPredictionConfig* config,
    const CranHostAnchor* anchor,
    const double* throttle_breakpoints,
    const double* velocity_breakpoints,
    std::size_t lookup_count,
    double* predicted_state);

}
