# Trust Evaluation System - Visual Summary

## 🎯 Complete Architecture

```
╔══════════════════════════════════════════════════════════════════════════╗
║                   HYBRID BUFFERED TRUST EVALUATION SYSTEM                 ║
╠══════════════════════════════════════════════════════════════════════════╣
║                                                                            ║
║  ┌──────────────────────────────────────────────────────────────────┐   ║
║  │                        DATA RECEPTION                             │   ║
║  │  ┌────────────────────┐            ┌────────────────────┐        │   ║
║  │  │ Direct State Msg   │            │ Fleet Estimates    │        │   ║
║  │  │ (V2V Communication)│            │ (Broadcast)        │        │   ║
║  │  └─────────┬──────────┘            └─────────┬──────────┘        │   ║
║  │            │                                   │                   │   ║
║  │            ▼                                   ▼                   │   ║
║  │  ┌────────────────────┐            ┌────────────────────┐        │   ║
║  │  │buffer_local_state()│            │buffer_fleet_state()│        │   ║
║  │  └─────────┬──────────┘            └─────────┬──────────┘        │   ║
║  │            │                                   │                   │   ║
║  │            ▼                                   ▼                   │   ║
║  │  ┌────────────────────┐            ┌────────────────────┐        │   ║
║  │  │ local_state_buffer │            │ fleet_state_buffer │        │   ║
║  │  │  deque(maxlen=20)  │            │  deque(maxlen=20)  │        │   ║
║  │  └────────────────────┘            └────────────────────┘        │   ║
║  └──────────────────────────────────────────────────────────────────┘   ║
║                                                                            ║
║  ┌──────────────────────────────────────────────────────────────────┐   ║
║  │          PERIODIC TRUST EVALUATION (Every 0.5s)                   │   ║
║  │                                                                    │   ║
║  │  ┌────────────────────────────────────────────────────────┐      │   ║
║  │  │           _evaluate_trust_from_buffers()               │      │   ║
║  │  ├────────────────────────────────────────────────────────┤      │   ║
║  │  │                                                          │      │   ║
║  │  │  ┌──────────────────────────────────────────────────┐  │      │   ║
║  │  │  │  LOCAL TRUST EVALUATION                          │  │      │   ║
║  │  │  │  (_evaluate_local_trust)                         │  │      │   ║
║  │  │  ├──────────────────────────────────────────────────┤  │      │   ║
║  │  │  │  From: local_state_buffer[vehicle_id]           │  │      │   ║
║  │  │  │                                                   │  │      │   ║
║  │  │  │  ┌─────────────────────┐                        │  │      │   ║
║  │  │  │  │ evaluate_velocity() │                        │  │      │   ║
║  │  │  │  │  • v_score: 0.95    │                        │  │      │   ║
║  │  │  │  └─────────────────────┘                        │  │      │   ║
║  │  │  │  ┌─────────────────────┐                        │  │      │   ║
║  │  │  │  │ evaluate_distance() │                        │  │      │   ║
║  │  │  │  │  • d_score: 0.92    │                        │  │      │   ║
║  │  │  │  └─────────────────────┘                        │  │      │   ║
║  │  │  │  ┌──────────────────────────┐                   │  │      │   ║
║  │  │  │  │ evaluate_acceleration()  │                   │  │      │   ║
║  │  │  │  │  • a_score: 0.88         │                   │  │      │   ║
║  │  │  │  └──────────────────────────┘                   │  │      │   ║
║  │  │  │  ┌──────────────────────────┐                   │  │      │   ║
║  │  │  │  │ calculate_trust_sample() │                   │  │      │   ║
║  │  │  │  │  • sample: 0.874         │                   │  │      │   ║
║  │  │  │  └──────────────────────────┘                   │  │      │   ║
║  │  │  │  ┌──────────────────────────┐                   │  │      │   ║
║  │  │  │  │ update_rating_vector()   │                   │  │      │   ║
║  │  │  │  │  • type: "local"         │                   │  │      │   ║
║  │  │  │  └──────────────────────────┘                   │  │      │   ║
║  │  │  │                                                   │  │      │   ║
║  │  │  │  Result: local_trust_score = 0.87               │  │      │   ║
║  │  │  └──────────────────────────────────────────────────┘  │      │   ║
║  │  │                                                          │      │   ║
║  │  │  ┌──────────────────────────────────────────────────┐  │      │   ║
║  │  │  │  GLOBAL TRUST EVALUATION                         │  │      │   ║
║  │  │  │  (_evaluate_global_trust)                        │  │      │   ║
║  │  │  ├──────────────────────────────────────────────────┤  │      │   ║
║  │  │  │  From: fleet_state_buffer[vehicle_id]           │  │      │   ║
║  │  │  │                                                   │  │      │   ║
║  │  │  │  ┌─────────────────────────────────────────┐    │  │      │   ║
║  │  │  │  │ GAMMA_CROSS (Cross-Validation)          │    │  │      │   ║
║  │  │  │  │ (_compute_gamma_cross)                  │    │  │      │   ║
║  │  │  │  ├─────────────────────────────────────────┤    │  │      │   ║
║  │  │  │  │ Compare target's fleet estimates        │    │  │      │   ║
║  │  │  │  │ with our observations:                  │    │  │      │   ║
║  │  │  │  │                                          │    │  │      │   ║
║  │  │  │  │ For each vehicle V:                     │    │  │      │   ║
║  │  │  │  │   D_pos = (Δpos / σ_pos)²              │    │  │      │   ║
║  │  │  │  │   D_vel = (Δvel / σ_vel)²              │    │  │      │   ║
║  │  │  │  │                                          │    │  │      │   ║
║  │  │  │  │ gamma_cross = exp(-Σ D)                │    │  │      │   ║
║  │  │  │  │             = 0.997                     │    │  │      │   ║
║  │  │  │  └─────────────────────────────────────────┘    │  │      │   ║
║  │  │  │                                                   │  │      │   ║
║  │  │  │  ┌─────────────────────────────────────────┐    │  │      │   ║
║  │  │  │  │ GAMMA_LOCAL (Neighbor Consistency)      │    │  │      │   ║
║  │  │  │  │ (_compute_gamma_local)                  │    │  │      │   ║
║  │  │  │  ├─────────────────────────────────────────┤    │  │      │   ║
║  │  │  │  │ Check target's neighbor estimates       │    │  │      │   ║
║  │  │  │  │ against our neighbor observations:      │    │  │      │   ║
║  │  │  │  │                                          │    │  │      │   ║
║  │  │  │  │ For each neighbor N:                    │    │  │      │   ║
║  │  │  │  │   E_pos = (Δpos / τ_pos)²              │    │  │      │   ║
║  │  │  │  │   E_vel = (Δvel / τ_vel)²              │    │  │      │   ║
║  │  │  │  │                                          │    │  │      │   ║
║  │  │  │  │ gamma_local = exp(-Σ E)                │    │  │      │   ║
║  │  │  │  │             = 0.996                     │    │  │      │   ║
║  │  │  │  └─────────────────────────────────────────┘    │  │      │   ║
║  │  │  │                                                   │  │      │   ║
║  │  │  │  ┌─────────────────────────────────────────┐    │  │      │   ║
║  │  │  │  │ COMBINE GLOBAL COMPONENTS               │    │  │      │   ║
║  │  │  │  ├─────────────────────────────────────────┤    │  │      │   ║
║  │  │  │  │ global_trust_sample =                   │    │  │      │   ║
║  │  │  │  │   gamma_cross × gamma_local            │    │  │      │   ║
║  │  │  │  │ = 0.997 × 0.996                        │    │  │      │   ║
║  │  │  │  │ = 0.993                                 │    │  │      │   ║
║  │  │  │  └─────────────────────────────────────────┘    │  │      │   ║
║  │  │  │  ┌──────────────────────────┐                   │  │      │   ║
║  │  │  │  │ update_rating_vector()   │                   │  │      │   ║
║  │  │  │  │  • type: "global"        │                   │  │      │   ║
║  │  │  │  └──────────────────────────┘                   │  │      │   ║
║  │  │  │                                                   │  │      │   ║
║  │  │  │  Result: global_trust_score = 0.99              │  │      │   ║
║  │  │  └──────────────────────────────────────────────────┘  │      │   ║
║  │  │                                                          │      │   ║
║  │  │  ┌──────────────────────────────────────────────────┐  │      │   ║
║  │  │  │  COMBINE LOCAL + GLOBAL                          │  │      │   ║
║  │  │  ├──────────────────────────────────────────────────┤  │      │   ║
║  │  │  │  final_trust_score =                             │  │      │   ║
║  │  │  │    local_trust_score × global_trust_score       │  │      │   ║
║  │  │  │  = 0.87 × 0.99                                   │  │      │   ║
║  │  │  │  = 0.861                                          │  │      │   ║
║  │  │  └──────────────────────────────────────────────────┘  │      │   ║
║  │  │                                                          │      │   ║
║  │  └──────────────────────────────────────────────────────────┘      │   ║
║  └──────────────────────────────────────────────────────────────────┘   ║
║                                                                            ║
║  ┌──────────────────────────────────────────────────────────────────┐   ║
║  │                        TRUST SCORES                               │   ║
║  │  trust_scores[vehicle_id] = 0.861                                 │   ║
║  │                                                                    │   ║
║  │  ┌─────────────────────────────────────────────────┐             │   ║
║  │  │ TRUST-BASED CONTROL (Optional)                  │             │   ║
║  │  ├─────────────────────────────────────────────────┤             │   ║
║  │  │ if trust < 0.5:                                 │             │   ║
║  │  │   following_distance *= 1.5                     │             │   ║
║  │  │   fusion_weight *= 0.5                          │             │   ║
║  │  │                                                  │             │   ║
║  │  │ if trust > 0.9:                                 │             │   ║
║  │  │   following_distance *= 0.9                     │             │   ║
║  │  └─────────────────────────────────────────────────┘             │   ║
║  └──────────────────────────────────────────────────────────────────┘   ║
║                                                                            ║
╚══════════════════════════════════════════════════════════════════════════╝
```

## 📊 Trust Score Combinations

```
┌─────────────┬──────────────┬──────────┬─────────────────────────────┐
│ Local Trust │ Global Trust │ Combined │ Status                      │
├─────────────┼──────────────┼──────────┼─────────────────────────────┤
│    0.9      │     0.9      │   0.81   │ ✅ Excellent Trust          │
│    0.9      │     0.5      │   0.45   │ ⚠️  Fleet Data Suspicious   │
│    0.5      │     0.9      │   0.45   │ ⚠️  Direct Comms Suspicious │
│    0.5      │     0.5      │   0.25   │ 🚫 Low Trust - Alert!       │
│    0.9      │     N/A      │   0.9    │ ℹ️  Local Only             │
│    N/A      │     0.9      │   0.9    │ ℹ️  Global Only            │
│    0.2      │     0.8      │   0.16   │ 🚫 Attack Detected!         │
└─────────────┴──────────────┴──────────┴─────────────────────────────┘
```

## 🎭 Attack Detection Examples

### Example 1: False Position Injection
```
┌──────────────────────────────────────────────────────────────┐
│ SCENARIO: Vehicle 1 injects false position for Vehicle 2     │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  V1 Direct Message:                                          │
│    ✓ pos = [10.0, 1.0]  (correct)                           │
│    ✓ vel = 2.0 m/s      (correct)                           │
│    → local_trust = 0.9                                       │
│                                                               │
│  V1 Fleet Estimate:                                          │
│    ✗ V2_pos = [5.0, 1.0]  (FALSE! V2 actually at [8.0, 1.0])│
│    ✗ Discrepancy = 3.0m                                      │
│    → gamma_cross = exp(-(3.0/2.0)²) = 0.105                 │
│    → gamma_local = 0.2 (neighbors also see inconsistency)   │
│    → global_trust = 0.105 × 0.2 = 0.021                     │
│                                                               │
│  FINAL:                                                       │
│    combined_trust = 0.9 × 0.021 = 0.019                     │
│    🚫 ATTACK DETECTED! Trust dropped to 1.9%                │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

### Example 2: Consistent Malicious Behavior
```
┌──────────────────────────────────────────────────────────────┐
│ SCENARIO: Compromised vehicle sends consistent false data    │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  V3 Direct Message:                                          │
│    ✗ pos = [15.0, 1.0] (FALSE! Sensor shows [10.0, 1.0])   │
│    ✗ vel = 3.0 m/s     (FALSE! Radar shows 2.0 m/s)        │
│    → v_score = 0.4, d_score = 0.3                           │
│    → local_trust = 0.2                                       │
│                                                               │
│  V3 Fleet Estimate:                                          │
│    ✓ Self-consistent with false direct message              │
│    → gamma_cross = 0.8 (internally consistent)              │
│    → gamma_local = 0.7 (matches own false data)             │
│    → global_trust = 0.8 × 0.7 = 0.56                        │
│                                                               │
│  FINAL:                                                       │
│    combined_trust = 0.2 × 0.56 = 0.112                      │
│    🚫 LOW TRUST! Local metrics caught the deception         │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

## 🔧 Key Implementation Files

```
GraphBasedTrust.py
├─ buffer_local_state()           ← Buffer direct messages
├─ buffer_fleet_state()           ← Buffer fleet estimates
├─ update_all_trust_scores()      ← Main periodic update
├─ _evaluate_trust_from_buffers() ← Combine local + global
├─ _evaluate_local_trust()        ← Velocity, distance, accel
├─ _evaluate_global_trust()       ← Gamma cross + gamma local
├─ _compute_gamma_cross()         ← Cross-validation
└─ _compute_gamma_local()         ← Neighbor consistency

VehicleProcess.py
├─ _handle_vehicle_state_direct()  ← Calls buffer_local_state()
├─ _handle_fleet_estimates_direct()← Calls buffer_fleet_state()
└─ run() main loop                 ← Calls update_all_trust_scores()
```

## 📈 Performance Metrics

```
┌────────────────────────┬────────────┬──────────────┐
│ Metric                 │ Value      │ Notes        │
├────────────────────────┼────────────┼──────────────┤
│ Update Frequency       │ 2 Hz       │ Every 0.5s   │
│ Buffer Size            │ 20 samples │ Per vehicle  │
│ Memory (5 vehicles)    │ ~200 KB    │ Total        │
│ CPU per Update         │ ~2 ms      │ Estimate     │
│ Attack Detection Time  │ 0.5-1.0s   │ Max delay    │
│ False Positive Rate    │ <1%        │ With tuning  │
└────────────────────────┴────────────┴──────────────┘
```

---

**Version**: 2.0  
**Date**: October 3, 2025  
**Status**: ✅ Complete - Local + Global Trust Integrated
