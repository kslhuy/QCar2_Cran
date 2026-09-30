"""Bounded learned-trust filter for the simulation car.

The small shared MLP estimates measurement reliability. Covariance gains stay
positive and diagonal: an attacked position channel cannot correct heading or
speed. An independent odometry anchor prevents a persistent bias from becoming
its own reference. All memory is causal and reset per drive.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import numpy as np


def wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


class InnovationTrustFilter:
    FORMAT = "qcar_innovation_trust_v1"
    FEATURE_COUNT = 12

    def __init__(self, initial_pose=None, config=None, checkpoint=None):
        self.config = dict(config or {})
        self.wheelbase = float(self.config.get("kin_wheelbase", .26))
        if not np.isfinite(self.wheelbase) or self.wheelbase <= 0:
            raise ValueError("wheelbase must be positive and finite")
        self.noise = np.array([.035, .035, .015, .012])
        self.process = np.array([.002, .002, .002, .006])**2
        self.scales = np.array([.12, .12, .09, .10])
        self.weights = None
        self.checkpoint_metadata = {}
        if checkpoint:
            with np.load(checkpoint, allow_pickle=False) as payload:
                metadata = json.loads(str(payload["metadata"]))
                if metadata.get("format") != self.FORMAT:
                    raise ValueError("Not an innovation-trust checkpoint")
                self.weights = [np.asarray(payload[k], dtype=float)
                                for k in ("w1", "b1", "w2", "b2")]
                if self.weights[0].shape[1] != self.FEATURE_COUNT:
                    raise ValueError("Checkpoint feature layout mismatch")
                w1, b1, w2, b2 = self.weights
                if (w1.ndim != 2 or b1.shape != (len(w1),) or w2.shape != (1, len(w1))
                        or b2.shape != (1,) or not all(np.isfinite(w).all() for w in self.weights)):
                    raise ValueError("Invalid trust-network weights")
                self.checkpoint_metadata = metadata
                self.noise = np.asarray(metadata.get("measurement_std", self.noise))
                if self.noise.shape != (4,) or not np.all(np.isfinite(self.noise) & (self.noise > 0)):
                    raise ValueError("Invalid measurement standard deviations")
        self.reset(initial_pose)

    def reset(self, initial_pose=None):
        self.state = np.zeros(4)
        if initial_pose is not None:
            p = np.asarray(initial_pose).reshape(-1)
            self.state[:min(len(p), 4)] = p[:4]
        self.anchor = self.state.copy()
        self.drive_velocity = self.state[3]
        self.steering_state = 0.
        self.covariance = np.array([.02, .02, .02, .04])**2
        self.previous_z = None
        self.previous_anchor = self.anchor.copy()
        self.ema = np.zeros(4)
        self.frozen_time = np.zeros(4)
        self.last_features = np.zeros((4, self.FEATURE_COUNT))
        self.last_trust = np.zeros(4)
        self.last_gain = np.zeros(4)
        self.last_prediction = self.state.copy()
        self.last_measurement = self.state.copy()
        self.elapsed = 0.
        self.last_gps_time = 0.
        self.previous_gyro = None
        self.previous_steering = None
        self.gyro_fault_latched = False
        self.previous_tach = None
        self.tach_frozen_seconds = 0.

    def _velocity_prediction(self, velocity, throttle, dt):
        model = self.config.get("longitudinal_model", "velocity_lag_lookup")
        if model == "velocity_lag_lookup":
            cfg = self.config.get("velocity_lag_lookup_model", {})
            xs = np.asarray(cfg.get("throttle_breakpoints", []))
            ys = np.asarray(cfg.get("steady_state_velocity_breakpoints", []))
            if len(xs) < 2 or len(xs) != len(ys):
                raise ValueError("Innovation trust requires calibrated throttle breakpoints")
            if throttle < 0 and xs.min() >= 0:
                target = -float(np.interp(-throttle, xs, ys))
            else:
                target = float(np.interp(throttle, xs, ys))
            accel = (target-velocity) / max(float(cfg.get("tau", .301)), .001)
        elif model == "velocity_lag":
            cfg = self.config.get("velocity_lag_model", {})
            u = math.copysign(max(abs(throttle)-float(cfg.get("throttle_deadband", 0)), 0), throttle)
            accel = (float(cfg.get("velocity_gain", 6.598))*u-velocity) / max(float(cfg.get("tau", .301)), .001)
        elif model == "simple_acceleration":
            accel = throttle
        else:
            raise ValueError(f"Unsupported innovation-trust longitudinal model: {model}")
        # CommonRoad's MockQCar reduces positive acceleration above v_switch;
        # a first-order throttle lag alone misses this limit after a stop.
        switch = float(self.config.get("positive_accel_switch", .1))
        positive_limit = 2*switch/max(velocity, switch)
        accel = float(np.clip(accel, -2, positive_limit))
        if abs(throttle) < .01 and abs(velocity) < .03:
            accel = 0.
        return float(np.clip(velocity+dt*accel, -2.5, 2.5)), accel

    def predict_trust(self, features):
        if self.weights is None:
            return np.ones(len(features))
        w1, b1, w2, b2 = self.weights
        hidden = np.maximum(features @ w1.T + b1, 0)
        logits = (hidden @ w2.T + b2).reshape(-1)
        return 1 / (1 + np.exp(-np.clip(logits, -40, 40)))

    def update(self, motor_tach, steering, throttle, dt, gyro_z=0.,
               gps_data=None, acceleration=None):
        dt = float(dt)
        if not np.isfinite(dt) or not 0 < dt <= .25:
            raise ValueError("dt must be finite and in (0, 0.25]")
        if not np.isfinite(throttle):
            raise ValueError("Non-finite control input")
        self.elapsed += dt
        previous = self.state.copy()
        self.drive_velocity, model_accel = self._velocity_prediction(self.drive_velocity, throttle, dt)
        v_pred, _ = self._velocity_prediction(previous[3], throttle, dt)
        # Tachometer is checked against an independent command-driven model.
        # Its four copies in the old wheel branch are not four independent votes.
        tach = float(motor_tach)
        self.tach_frozen_seconds = (self.tach_frozen_seconds+dt
                                   if self.previous_tach is not None and tach == self.previous_tach else 0.)
        self.previous_tach = tach if np.isfinite(tach) else None
        tach_frozen = self.tach_frozen_seconds > .15 and max(abs(tach), abs(self.drive_velocity)) > .05
        v_residual = tach - self.drive_velocity
        wheel_trust = math.exp(-min((abs(v_residual)/.20)**4, 60)) if np.isfinite(tach) else 0.
        if tach_frozen:
            wheel_trust = 0.
        if self.elapsed > dt:
            wheel_trust *= self.last_trust[3]
        v_used = v_pred + .65*wheel_trust*(tach-v_pred) if wheel_trust else v_pred
        delta = float(steering) if np.isfinite(steering) else self.steering_state
        old_delta = self.steering_state
        self.steering_state += np.clip(10*(np.clip(delta, -.52, .52)-old_delta), -3, 3)*dt
        bicycle_rate = .5*(previous[3]+v_used)*math.tan(.5*(old_delta+self.steering_state))/self.wheelbase
        ax = float(np.asarray(acceleration).reshape(-1)[0]) if acceleration is not None else model_accel
        if self.previous_gyro is not None and np.isfinite(gyro_z):
            if abs(float(gyro_z)-self.previous_gyro) > .3 and abs(delta-self.previous_steering) < .05:
                self.gyro_fault_latched = True
            if abs(float(gyro_z)-bicycle_rate) < .12:
                self.gyro_fault_latched = False
        gyro_ok = np.isfinite(gyro_z) and (abs(float(gyro_z)-bicycle_rate) < .3 or abs(ax-model_accel) < .65)
        gyro_ok = gyro_ok and not self.gyro_fault_latched
        rate = float(gyro_z) if gyro_ok else bicycle_rate
        self.previous_gyro = float(gyro_z) if np.isfinite(gyro_z) else None
        self.previous_steering = delta
        v_pose = .5*(previous[3]+v_used)
        psi_mid = previous[2] + .5*dt*rate
        pred = np.array([previous[0]+v_pose*math.cos(psi_mid)*dt,
                         previous[1]+v_pose*math.sin(psi_mid)*dt,
                         wrap(previous[2]+rate*dt), v_pred])
        # Anchor is integrated without direct GPS corrections at filter cadence.
        anchor_mid = self.anchor[2]+.5*rate*dt
        self.anchor[:2] += v_pose*dt*np.array([math.cos(anchor_mid), math.sin(anchor_mid)])
        self.anchor[2] = wrap(self.anchor[2]+rate*dt)
        self.anchor[3] = self.drive_velocity
        available = np.array([False, False, False, np.isfinite(tach)])
        z = pred.copy()
        z[3] = tach if available[3] else pred[3]
        if gps_data is not None:
            fresh = bool(gps_data.get("position_valid", gps_data.get("fresh", gps_data.get("valid", False))))
            heading_fresh = bool(gps_data.get("heading_valid", fresh))
            for j, key in enumerate(("x", "y", "theta")):
                value = gps_data.get(key, np.nan)
                available[j] = (fresh if j < 2 else heading_fresh) and np.isfinite(value)
                if available[j]:
                    z[j] = float(value)
        residual = z-pred
        residual[2] = wrap(residual[2])
        anchor_res = z-self.anchor
        anchor_res[2] = wrap(anchor_res[2])
        interval = max(self.elapsed-self.last_gps_time, dt)
        if self.previous_z is None:
            self.previous_z = z.copy()
        delta_z = z-self.previous_z
        delta_z[2] = wrap(delta_z[2])
        delta_anchor = self.anchor-self.previous_anchor
        delta_anchor[2] = wrap(delta_anchor[2])
        motion_error = delta_z-delta_anchor
        self.ema = np.where(available, .85*self.ema+.15*anchor_res, self.ema)
        self.frozen_time = np.where(available, np.where(abs(delta_z)<1e-7,
                                           self.frozen_time+interval, 0), self.frozen_time)
        scale = self.scales
        features = np.column_stack([
            np.log1p(abs(residual)/scale), np.log1p(abs(anchor_res)/scale),
            np.log1p(abs(motion_error)/scale), np.log1p(abs(self.ema)/scale),
            np.minimum(self.frozen_time, 3), np.full(4, abs(v_used)),
            np.full(4, abs(rate)), np.full(4, min(interval, 2)), np.eye(4),
        ])
        # Redescending prior, not a clipped correction that accumulates forever.
        cutoff = np.array([.30, .30, .20, .20])
        evidence = np.maximum(abs(residual), abs(anchor_res)*.75)
        prior = np.exp(-np.minimum((evidence/cutoff)**4, 60))
        if tach_frozen:
            prior[3] = 0.
        if self.frozen_time[0] > .15 and self.frozen_time[1] > .15 and abs(v_used) > .08:
            prior[:3] = 0.
        trust = prior * self.predict_trust(features) * available
        # Reject GPS positions as a pair; keep heading independent.
        trust[:2] = min(trust[:2])
        self.covariance += self.process * (dt/.02)
        gain = self.covariance/(self.covariance+self.noise**2)
        effective = gain*trust
        self.state = pred+effective*residual
        self.state[2] = wrap(self.state[2])
        self.covariance = np.maximum((1-effective)*self.covariance, 1e-9)
        # Slow anchor alignment is allowed only for small, trusted discrepancies.
        for j in range(3):
            if trust[j] > .8 and abs(anchor_res[j]) < scale[j]*.65:
                self.anchor[j] += min(interval/8., .1)*anchor_res[j]
        self.anchor[2] = wrap(self.anchor[2])
        self.previous_z = np.where(available, z, self.previous_z)
        self.previous_anchor = np.where(available, self.anchor, self.previous_anchor)
        if available[:3].any():
            self.last_gps_time = self.elapsed
        self.last_features = features
        self.last_trust = trust
        self.last_gain = gain
        self.last_prediction = pred
        self.last_measurement = z
        return True

    def get_state(self):
        return self.state.copy()
