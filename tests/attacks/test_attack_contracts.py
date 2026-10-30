"""Offline attack contracts: no sockets, hardware or live vehicle commands."""
import copy
import logging
from pathlib import Path
from threading import RLock
from types import SimpleNamespace
from unittest.mock import patch
from unittest.mock import Mock

import msgpack
import numpy as np
import pytest

from qcar.V2V.AttackModule.AttackModule import AttackModule
from qcar.V2V.AttackModule.AttackScenarios import make_scenario, load_scenarios_from_config
from qcar.V2V.AttackModule.V2VAttackInjector import V2VAttackInjector
from qcar.V2V.v2v_communication import V2VCommunication
from qcar.V2V.v2v_manager import V2VManager
from qcar.Observer.KalmaNet.Robust.sensor_attack_augmentation import (
    RuntimeAttackConfig, RuntimeSensorAttackSimulator,
)


def make_link(probability=0.5, channel="fleet"):
    frames = []
    clock = [1_800_000_000_000_000_000]
    comm = V2VCommunication.__new__(V2VCommunication)
    comm.vehicle_id = 0
    comm._is_active = True
    comm.send_socket = object()  # Deliberately not a socket.
    comm._lock = RLock()
    comm.send_intervals = {"local_state": 40_000_000, "fleet_state": 100_000_000}
    comm.message_send_times_ns = {}
    comm.sequence_number = 0
    comm.peer_vehicles = [1]
    comm.peer_ips = {1: "offline"}
    comm.peer_ports = {1: 0}
    comm.datagram_adapter = None
    comm.logger = logging.getLogger("attack-contracts")
    comm.stats = {"messages_sent": 0}
    def record(payload, address):
        frames.append(msgpack.unpackb(payload, raw=False, strict_map_key=False))
        return True
    comm._send_raw_datagram = record
    state = dict(vehicle_id=0, x=1., y=0., theta=0., velocity=.5, acceleration=.1)
    manager = SimpleNamespace(
        vehicle_id=0, logger=comm.logger, v2v_communication=comm, _lock=RLock(),
        stats={"local_broadcasts": 0, "fleet_broadcasts": 0},
        vehicle_observer=SimpleNamespace(
            get_local_state_for_broadcast=lambda: copy.deepcopy(state),
            get_fleet_state_for_broadcast=lambda: {"sender_id": 0, "fleet_states": {0: copy.deepcopy(state)}},
        ),
        _normalize_v2v_channel=V2VManager._normalize_v2v_channel,
        _broadcast_heartbeat=lambda: False, _broadcast_trust_report=lambda: False,
        _process_received_messages=lambda: None,
    )
    manager._build_dual_channel_payload = lambda **kw: V2VManager._build_dual_channel_payload(manager, **kw)
    attacks = AttackModule(0, random_seed=815)
    attacks.add_scenario(make_scenario(0, -1, 0., 100., "drop", probability,
                                      channel, ["all"], attack_type="DoS"))
    injector = V2VAttackInjector(manager, attack_module=attacks, start_time=0.)
    injector.get_elapsed_time = lambda: (clock[0] - 1_800_000_000_000_000_000) / 1e9
    return injector, comm, clock, frames


def exercise_link(probability, tick_ns=20_000_000, duration_ns=20_000_000_000):
    injector, comm, clock, frames = make_link(probability)
    with patch("qcar.V2V.v2v_communication.time.time_ns", side_effect=lambda: clock[0]):
        for tick in range(0, duration_ns, tick_ns):
            clock[0] = 1_800_000_000_000_000_000 + tick
            injector.update_broadcast()
    fleet = [frame for frame in frames if frame["message_type"] == "fleet_state"]
    return len(fleet), injector.attack_module.stats["fleet_attacks"]


def test_fleet_drop_consumes_one_draw_per_packet_slot():
    sent, dropped = exercise_link(.5)
    assert sent + dropped == 200
    assert 70 <= sent <= 130


def test_drop_stream_does_not_depend_on_control_poll_frequency():
    assert exercise_link(.5, 20_000_000) == exercise_link(.5, 1_000_000)


def test_disabled_injector_reports_inactive_attack():
    injector, _, clock, _ = make_link()
    with patch("qcar.V2V.v2v_communication.time.time_ns", side_effect=lambda: clock[0]):
        injector.update_broadcast()
    injector.disable_attacks()
    assert not injector.is_attack_active()
    assert not injector.get_attack_status()["attack_active"]
    assert injector.get_attack_status()["active_scenarios"] == 0
    assert injector.get_attack_status()["attack_value_snapshot"]["by_vehicle"] == {}


def test_unknown_field_is_rejected_instead_of_counted_as_an_attack():
    module = AttackModule(0)
    with pytest.raises(ValueError, match="field"):
        module.add_scenario(make_scenario(0, -1, 0., 1., "bias", 2., "local", ["veloctiy"]))


@pytest.mark.parametrize("value", [float("nan"), -0.1, 1.1])
def test_invalid_drop_probability_is_rejected(value):
    with pytest.raises(ValueError, match="probability"):
        module = AttackModule(0)
        module.add_scenario(make_scenario(0, -1, 0., 1., "drop", value, "local", ["all"]))


def test_bad_yaml_is_not_partially_loaded(tmp_path):
    path = tmp_path / "attacks.yaml"
    path.write_text("""attack_settings: {enabled: true}
scenarios:
  - {name: valid, t_start: 0, t_end: 1, attacker_id: 0, attack_type: Bogus, modification_type: bias, data_type: local, intensity: 1, target_fields: [x]}
  - {name: broken, t_start: 0, t_end: 1, attacker_id: 0, attack_type: Bogus, modification_type: bias, data_type: local, intensity: 1, target_fields: [veloctiy]}
""", encoding="utf-8")
    module = AttackModule(0)
    with pytest.raises(ValueError, match="broken"):
        load_scenarios_from_config(module, str(path))
    assert module.scenarios == []


def test_reacquisition_never_invents_a_fresh_gps_sample():
    simulator = RuntimeSensorAttackSimulator(RuntimeAttackConfig(
        attack_prob=0., gps_attack_prob=1., force_gps_attack=True,
        immediate_attack=True, gps_attack_types=["reacquisition"],
        min_attack_steps=3, max_attack_steps=3, seed=4,
    ))
    for tick in range(3):
        gps = dict(x=1., y=2., valid=True, position_valid=True, fresh=True) if tick == 0 else None
        output = simulator.apply(motor_tach=0., steering=0., gyro_z=0., throttle=0.,
                                 dt=.02, acceleration=np.zeros(3), gps_data=gps)
    assert not output[5]["fresh"]
    assert not output[5]["position_valid"]


@pytest.mark.parametrize("config", [
    {"attack_prob": 1.01}, {"gps_attack_prob": float("nan")},
    {"noise_std_range": (-1., .5)}, {"ramp_rate_range": (.2, .1)},
])
def test_runtime_sensor_parameters_are_validated(config):
    with pytest.raises(ValueError):
        RuntimeSensorAttackSimulator(RuntimeAttackConfig.from_dict(config))


def test_runtime_typo_is_rejected():
    with pytest.raises(ValueError, match="unknown|Unknown"):
        RuntimeAttackConfig.from_dict({"gps_attack_probb": 1.0})


@pytest.mark.parametrize("channel,family,fields,victims", [
    ("heartbeat", "Bogus", ["velocity"], [-1]),
    ("local", "Replay", ["x"], [-1]),
    ("local", "Delay", ["x"], [-1]),
    ("local", "Bogus", ["x"], [2]),
])
def test_unimplemented_attack_contract_is_explicit(channel, family, fields, victims):
    with pytest.raises(ValueError, match="implemented"):
        AttackModule(0).add_scenario(make_scenario(0, victims, 0., 1., "bias", 1., channel, fields, attack_type=family))


def test_missing_payload_field_does_not_increment_applied_counter():
    module = AttackModule(0)
    module.add_scenario(make_scenario(0, -1, 0., 1., "bias", 2., "local", ["acceleration"]))
    module.update(.1)
    assert module.apply_attack_to_local_state({"x": 1.}) == {"x": 1.}
    assert module.stats["total_attacks_applied"] == 0


def test_zero_probability_sensor_branch_is_never_selected():
    simulator = RuntimeSensorAttackSimulator(RuntimeAttackConfig(
        branch_attack_probs={"imu": .0001, "steer": 0., "wheel": 0.}, seed=4))
    assert all(simulator._pick_branches_to_attack() == ["imu"] for _ in range(100))


def test_dropout_age_uses_elapsed_time_and_does_not_invent_a_prior_fix():
    simulator = RuntimeSensorAttackSimulator(RuntimeAttackConfig(
        attack_prob=0., gps_attack_prob=1., force_gps_attack=True, immediate_attack=True,
        gps_attack_types=["dropout"], min_attack_steps=10, max_attack_steps=10, seed=2))
    values = dict(motor_tach=0., steering=0., gyro_z=0., throttle=0., acceleration=np.zeros(3))
    first = simulator.apply(**values, dt=.01, gps_data=None)[5]
    assert not first["hold_valid"]
    simulator.reset()
    simulator.apply(**values, dt=.01, gps_data={"x": 1., "y": 2., "valid": True})
    simulator.apply(**values, dt=.03, gps_data=None)
    output = simulator.apply(**values, dt=.02, gps_data=None)[5]
    assert output["age_sec"] == pytest.approx(.05)
    assert output["hold_valid"]


def test_timed_trigger_expires_without_the_client_stop_message():
    from qcar.vehicle_logic import VehicleLogic
    injector, _, clock, _ = make_link()
    logic = VehicleLogic.__new__(VehicleLogic)
    logic.vehicle_id = 0
    logic.v2v_manager = injector
    logic.vehicle_logger = SimpleNamespace(logger=logging.getLogger("timed-attack"), log_error=Mock())
    logic._get_v2v_log_time_s = lambda: 0.
    logic._sync_v2v_attack_status_to_observer = Mock()
    logic.trigger_v2v_attack({"attack_type": "Bogus", "case_num": 1,
                              "attacker_id": 0, "victim_ids": [-1], "duration_s": 1.5})
    assert len(injector.attack_module.scenarios) == 1
    assert injector.attack_module.scenarios[0].t_end == 1.5
    with patch("qcar.V2V.v2v_communication.time.time_ns", side_effect=lambda: clock[0]):
        injector.update_broadcast()
        assert injector.is_attack_active()
        clock[0] += 2_000_000_000
        injector.update_broadcast()
        assert not injector.is_attack_active()
    logic.vehicle_logger.log_error.assert_not_called()


def test_timed_cli_carries_vehicle_side_expiry():
    from qcar.simulation.timed_v2v_attack import _build_parser, _build_trigger_payload
    args = _build_parser().parse_args(["--duration", "3.5"])
    assert _build_trigger_payload(args)["duration_s"] == 3.5
    args.no_disable = True
    assert "duration_s" not in _build_trigger_payload(args)


def test_combined_drop_and_falsification_counts_only_applied_changes():
    module = AttackModule(0, random_seed=1)
    module.add_scenario(make_scenario(0, -1, 0., 1., "drop", 0., "local", ["all"]))
    module.add_scenario(make_scenario(0, -1, 0., 1., "bias", .1, "local", ["velocity"]))
    module.update(.1)
    assert not module.should_drop_local_message()
    assert module.apply_attack_to_local_state({"velocity": .5})["velocity"] == pytest.approx(.6)
    assert module.stats["total_attacks_applied"] == 1


def test_fractional_step_requires_a_finite_duration():
    with pytest.raises(ValueError, match="finite"):
        AttackModule(0).add_scenario(make_scenario(0, -1, 0., float("inf"),
            "step", {"at": .5, "value": 1.}, "local", ["x"]))


def test_training_aliases_and_branch_selection_preserve_clean_inputs():
    import torch
    from qcar.Observer.KalmaNet.Robust.sensor_attack_augmentation import (
        AttackConfig, SensorAttackAugmenter, BRANCH_KEYS,
    )
    config = AttackConfig.from_dict(dict(attack_types=["bias"], attack_mode="mixed",
        attack_prob=1., meas_attack_prob=0., direct_meas_attack_prob=0., gps_attack_prob=0.,
        branch_attack_probs={"imu": .0001, "steer": 0., "wheel": 0.}))
    raw = {key: torch.ones(2, 16) for keys in BRANCH_KEYS.values() for key in keys}
    z = torch.zeros(2, 16, 4)
    changed, observed, branches, measurements = SensorAttackAugmenter(config).augment_batch(raw, z)
    assert torch.all(branches[:, 0] == 1) and torch.all(branches[:, 1:] == 0)
    for key in raw:
        assert torch.all(raw[key] == 1)
        if key in BRANCH_KEYS["imu"]:
            assert not torch.equal(changed[key], raw[key])
        else:
            assert torch.equal(changed[key], raw[key])
    assert torch.equal(observed, z) and not measurements.any()


def test_legacy_runtime_gps_mode_selects_only_gps():
    from qcar.Observer.KalmaNet.Robust.runtime_sensor_attack import RuntimeSensorAttackMixin
    values = RuntimeSensorAttackMixin._normalize_runtime_attack_config(
        {"attack_mode": "gps", "gps_attack_types": ["jump"]})
    assert values["attack_prob"] == 0.
    assert values["gps_attack_prob"] == 1.
    assert values["forced_branches"] == []
    assert values["force_gps_attack"]
