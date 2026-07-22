"""Unit tests for fleet contracts, state machine, peer exchange, and scenarios."""

from __future__ import annotations

import os
from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.commands import CommandType, VehicleCommand
from core.types import V2VMessage, VehicleStateEstimate
from extra.simulator.virtual.process_runner import VirtualProcessManager
from extra.simulator.virtual.scenario import load_virtual_setup
from utils.fleet import (
    FleetError,
    FleetFormationBuilder,
    FleetManager,
    FleetPeerStore,
    FleetStateMachine,
    FleetMember,
    FleetPhase,
    FleetPolicy,
    FleetPeerSnapshot,
    FleetRegistry,
    DistributedEstimateSource,
    DistributedObserverFake,
    DistributedObserverLuenberger,
    VEHICLE_STATE_ESTIMATE,
    decode_vehicle_state_estimate,
    encode_vehicle_state_estimate,
)


def _policy(
    topology: str = "predecessor_chain",
    following_policy: str = "direct_predecessor",
    follower_controller_profile: str | None = None,
) -> FleetPolicy:
    return FleetPolicy.from_mapping(
        {
            "following_policy": following_policy,
            **(
                {"follower_controller_profile": follower_controller_profile}
                if follower_controller_profile is not None
                else {}
            ),
            "communication": {
                "topology": topology,
                "edge_direction": "directed",
                "ego_estimate_rate_hz": 20,
                "peer_timeout_s": 0.5,
            },
        }
    )


def _member(vehicle_id: int, role: str, order: int) -> FleetMember:
    return FleetMember.from_mapping(
        {"vehicle_id": vehicle_id, "role": role, "member_order": order}
    )


def _estimate(timestamp: float = 1.0) -> VehicleStateEstimate:
    return VehicleStateEstimate(timestamp, 1.0, 2.0, 0.2, 0.5, 0.1, True)


def _message(sender_id: int, sequence: int, payload: dict, received_at: float) -> V2VMessage:
    return V2VMessage(
        sender_id=sender_id,
        message_type=VEHICLE_STATE_ESTIMATE,
        payload=payload,
        sequence=sequence,
        sent_at_monotonic=0.0,
        sent_at_perf_counter_ns=0,
        received_at_monotonic=received_at,
        received_at_perf_counter_ns=0,
    )


class TestFleetFormation(unittest.TestCase):
    def test_builder_uses_member_order_not_yaml_order(self):
        formation = FleetFormationBuilder().build(
            "ordered",
            (
                _member(3, "follower", 2),
                _member(1, "leader", 0),
                _member(2, "follower", 1),
            ),
            _policy(),
            available_vehicle_ids=(1, 2, 3),
        )

        self.assertEqual([member.vehicle_id for member in formation.members], [1, 2, 3])
        self.assertEqual(formation.predecessor(3).vehicle_id, 2)
        self.assertEqual(formation.outbound_peer_ids(2), (3,))
        self.assertEqual(formation.inbound_peer_ids(2), (1,))
        self.assertEqual(formation.inbound_peer_ids(1), ())

    def test_direct_predecessor_rejects_leader_star_for_three_members(self):
        with self.assertRaisesRegex(FleetError, "direct predecessor route"):
            FleetFormationBuilder().build(
                "invalid_star",
                (
                    _member(1, "leader", 0),
                    _member(2, "follower", 1),
                    _member(3, "follower", 2),
                ),
                _policy("leader_follower"),
            )

    def test_transport_validation_requires_configured_topology_routes(self):
        formation = FleetFormationBuilder().build(
            "routes",
            (_member(1, "leader", 0), _member(2, "follower", 1)),
            _policy(),
        )
        builder = FleetFormationBuilder()

        with self.assertRaisesRegex(FleetError, "missing V2V routes"):
            builder.validate_transport_routes(formation, {1: (), 2: (1,)})
        builder.validate_transport_routes(formation, {1: (2,), 2: (1,)})

    def test_non_member_cannot_query_formation(self):
        formation = FleetFormationBuilder().build(
            "members",
            (_member(1, "leader", 0), _member(2, "follower", 1)),
            _policy(),
        )

        with self.assertRaisesRegex(FleetError, "not a member"):
            formation.member(3)


class TestFleetRegistryAndStateMachine(unittest.TestCase):
    def setUp(self):
        formation = FleetFormationBuilder().build(
            "registry",
            (_member(1, "leader", 0), _member(2, "follower", 1)),
            _policy("vehicle_vehicle"),
        )
        self.registry = FleetRegistry(formation)

    def test_join_and_leave_create_revisioned_snapshots(self):
        joined = self.registry.join(_member(3, "follower", 2))
        self.assertEqual(joined.membership_revision, 1)
        self.assertEqual([member.vehicle_id for member in joined.members], [1, 2, 3])

        left = self.registry.leave(3)
        self.assertEqual(left.membership_revision, 2)
        self.assertEqual([member.vehicle_id for member in left.members], [1, 2])

    def test_rejects_membership_change_that_breaks_formation(self):
        with self.assertRaisesRegex(FleetError, "one leader and at least one follower"):
            self.registry.leave(2)
        with self.assertRaisesRegex(FleetError, "already a fleet member"):
            self.registry.join(_member(2, "follower", 1))

    def test_lifecycle_build_activate_cancel_and_fault(self):
        lifecycle = FleetStateMachine(self.registry, vehicle_id=1)
        self.assertFalse(lifecycle.request_build(vehicle_running=False))
        self.assertTrue(lifecycle.request_build(vehicle_running=True))
        self.assertEqual(lifecycle.phase, FleetPhase.BUILDING)
        self.assertTrue(lifecycle.activate())
        self.assertEqual(lifecycle.status().source_vehicle_id, 1)
        self.assertEqual(lifecycle.status().membership_revision, 0)

        self.assertTrue(lifecycle.fault("peer timeout"))
        self.assertEqual(lifecycle.phase, FleetPhase.CANCELLING)
        self.assertTrue(lifecycle.complete_cancellation())
        self.assertEqual(lifecycle.phase, FleetPhase.DISABLED)

    def test_peer_snapshot_is_a_fleet_owned_contract(self):
        snapshot = FleetPeerSnapshot(
            source_vehicle_id=2,
            member_order=1,
            role=_member(2, "follower", 1).role,
            membership_revision=0,
            estimate=VehicleStateEstimate(1.0, 2.0, 3.0, 0.1, 0.4, 0.0, True),
            source_sequence=4,
            source_timestamp=1.0,
            received_at_monotonic=2.0,
        )

        self.assertEqual(snapshot.source_vehicle_id, 2)
        self.assertTrue(snapshot.valid)


class TestFleetScenario(unittest.TestCase):
    def test_virtual_fleet_scenario_loads_profile_membership_and_routes(self):
        project_root = Path(__file__).resolve().parents[1]
        setup = load_virtual_setup(project_root / "config" / "scenarios" / "virtual_two_vehicle_fleet.yaml")

        self.assertIsNotNone(setup.fleet)
        assert setup.fleet is not None
        formation = setup.fleet.formation
        self.assertEqual(formation.formation_id, "virtual_two_vehicle_fleet")
        self.assertEqual(formation.predecessor(2).vehicle_id, 1)
        self.assertEqual(formation.outbound_peer_ids(1), (2,))

    def test_process_manager_attaches_a_local_lifecycle_for_fleet_member(self):
        project_root = Path(__file__).resolve().parents[1]
        manager = VirtualProcessManager(
            project_root / "config" / "scenarios" / "virtual_two_vehicle_fleet.yaml",
            vehicle_id=1,
        )
        context = manager.prepare()
        runtime = manager.build_runtime(context)
        try:
            self.assertIsNotNone(runtime.fleet)
            assert runtime.fleet is not None
            self.assertEqual(runtime.fleet.status().source_vehicle_id, 1)
            self.assertEqual(runtime.fleet.phase, FleetPhase.DISABLED)
        finally:
            runtime.shutdown()


class TestFleetPeerExchange(unittest.TestCase):
    def setUp(self):
        formation = FleetFormationBuilder().build(
            "exchange",
            (_member(1, "leader", 0), _member(2, "follower", 1)),
            _policy(),
        )
        self.registry = FleetRegistry(formation)
        self.leader = FleetStateMachine(self.registry, vehicle_id=1)

    def test_codec_round_trip_preserves_estimate_and_membership_identity(self):
        payload = encode_vehicle_state_estimate(_estimate(3.0), self.leader.status())
        snapshot = decode_vehicle_state_estimate(_message(1, 7, payload, received_at=4.0))

        self.assertEqual(snapshot.source_vehicle_id, 1)
        self.assertEqual(snapshot.member_order, 0)
        self.assertEqual(snapshot.role.value, "leader")
        self.assertEqual(snapshot.membership_revision, 0)
        self.assertEqual(snapshot.source_sequence, 7)
        self.assertEqual(snapshot.estimate.x, 1.0)

    def test_store_rejects_malformed_wrong_role_and_out_of_order_messages(self):
        store = FleetPeerStore(self.registry, vehicle_id=2)
        valid = encode_vehicle_state_estimate(_estimate(), self.leader.status())
        wrong_role = dict(valid)
        wrong_role["role"] = "follower"
        malformed = {"schema_version": 99}
        obsolete = dict(valid)
        obsolete["membership_revision"] = 1

        store.ingest(
            (
                _message(1, 4, valid, 1.0),
                _message(1, 5, wrong_role, 1.1),
                _message(1, 3, valid, 1.2),
                _message(1, 6, malformed, 1.3),
                _message(9, 1, valid, 1.4),
                _message(1, 7, obsolete, 1.5),
                _message(1, 7, valid, 1.6),
            )
        )

        self.assertEqual(store.predecessor_snapshot().source_sequence, 7)
        self.assertEqual(store.counters()["accepted"], 2)
        self.assertEqual(store.counters()["wrong_role"], 1)
        self.assertEqual(store.counters()["out_of_order"], 1)
        self.assertEqual(store.counters()["malformed"], 1)
        self.assertEqual(store.counters()["unexpected"], 1)
        self.assertEqual(store.counters()["obsolete_membership"], 1)
        self.assertEqual(store.counters()["sequence_gap"], 2)

    def test_store_removes_stale_snapshots_using_local_receive_time(self):
        store = FleetPeerStore(self.registry, vehicle_id=2)
        payload = encode_vehicle_state_estimate(_estimate(), self.leader.status())
        store.ingest((_message(1, 1, payload, 10.0),))
        store.prune_stale(10.51)

        self.assertIsNone(store.predecessor_snapshot())
        self.assertFalse(store.all_expected_fresh())
        self.assertEqual(store.counters()["stale_removed"], 1)

    def test_invalid_peer_estimate_removes_the_previous_control_reference(self):
        store = FleetPeerStore(self.registry, vehicle_id=2)
        valid = encode_vehicle_state_estimate(_estimate(), self.leader.status())
        invalid = dict(valid)
        invalid["valid"] = False

        store.ingest((_message(1, 1, valid, 1.0),))
        self.assertIsNotNone(store.predecessor_snapshot())
        store.ingest((_message(1, 2, invalid, 1.1),))

        self.assertIsNone(store.predecessor_snapshot())
        self.assertEqual(store.counters()["invalid"], 1)

    def test_managers_activate_after_exchange_and_fault_when_peer_expires(self):
        leader = FleetManager(self.registry, vehicle_id=1)
        follower = FleetManager(self.registry, vehicle_id=2)
        self.assertTrue(leader.request_build(vehicle_running=True, now_monotonic=0.0))
        self.assertTrue(follower.request_build(vehicle_running=True, now_monotonic=0.0))
        leader_publication = leader.build_publication(_estimate(), 0.0)
        follower_publication = follower.build_publication(_estimate(), 0.0)
        assert leader_publication is not None
        assert follower_publication is not None

        self.assertIsNone(leader.process_received((_message(2, 1, follower_publication.payload, 0.1),), 0.1))
        self.assertIsNone(follower.process_received((_message(1, 1, leader_publication.payload, 0.1),), 0.1))
        self.assertEqual(leader.phase, FleetPhase.ACTIVE)
        self.assertEqual(follower.phase, FleetPhase.ACTIVE)
        self.assertIsNone(follower.build_publication(_estimate(), 0.01))

        self.assertEqual(follower.process_received((), 0.61), "fleet peer became stale")

    def test_manager_rejects_an_invalid_local_observer_estimate(self):
        manager = FleetManager(self.registry, vehicle_id=1)
        self.assertTrue(manager.request_build(vehicle_running=True, now_monotonic=0.0))

        self.assertEqual(
            manager.process_ego_estimate(replace(_estimate(), valid=False)),
            "fleet local observer estimate invalid",
        )

    def test_manager_reports_build_timeout_when_a_required_peer_never_arrives(self):
        manager = FleetManager(self.registry, vehicle_id=2)
        self.assertTrue(manager.request_build(vehicle_running=True, now_monotonic=0.0))

        result = manager.step(_estimate(), (), now_monotonic=0.51, dt=0.05)

        self.assertEqual(result.fault_reason, "fleet peer build timeout")
        self.assertEqual(result.status.phase, FleetPhase.BUILDING)

    def test_manager_owns_fleet_lifecycle_command_interpretation(self):
        manager = FleetManager(self.registry, vehicle_id=2)

        build = manager.handle_command(VehicleCommand(CommandType.BUILD_FLEET), vehicle_running=True, now_monotonic=0.0)
        self.assertTrue(build.handled)
        self.assertFalse(build.stop_vehicle)
        self.assertEqual(manager.phase, FleetPhase.BUILDING)

        cancel = manager.handle_command(VehicleCommand(CommandType.CANCEL_FLEET), vehicle_running=True)
        self.assertTrue(cancel.handled)
        self.assertTrue(cancel.stop_vehicle)
        self.assertEqual(cancel.reason, "fleet_cancel")
        self.assertEqual(manager.phase, FleetPhase.DISABLED)

    def test_manager_run_cycle_owns_generic_v2v_drain_and_publication(self):
        class _Transport:
            def __init__(self, messages):
                self.messages = list(messages)
                self.publications = []

            def drain_received(self):
                messages, self.messages = self.messages, []
                return messages

            def publish(self, message_type, payload, target_vehicle_ids):
                self.publications.append((message_type, payload, target_vehicle_ids))

        manager = FleetManager(self.registry, vehicle_id=1)
        follower = FleetStateMachine(self.registry, vehicle_id=2)
        self.assertTrue(manager.request_build(vehicle_running=True, now_monotonic=0.0))
        payload = encode_vehicle_state_estimate(_estimate(), follower.status())
        transport = _Transport((_message(2, 1, payload, 0.1),))
        manager.attach_transport(transport)

        result = manager.run_cycle(_estimate(), now_monotonic=0.1, dt=0.05)

        self.assertEqual(result.status.phase, FleetPhase.ACTIVE)
        self.assertEqual(transport.publications[0][0], VEHICLE_STATE_ESTIMATE)
        self.assertEqual(transport.publications[0][2], [2])

    def test_active_follower_requests_the_policy_controller_profile(self):
        formation = FleetFormationBuilder().build(
            "profile-selection",
            (_member(1, "leader", 0), _member(2, "follower", 1)),
            _policy(follower_controller_profile="fleet_following"),
        )
        leader = FleetManager(FleetRegistry(formation), vehicle_id=1)
        follower = FleetManager(FleetRegistry(formation), vehicle_id=2)
        self.assertTrue(leader.request_build(vehicle_running=True, now_monotonic=0.0))
        self.assertTrue(follower.request_build(vehicle_running=True, now_monotonic=0.0))
        leader_result = leader.step(_estimate(), (), now_monotonic=0.1)
        payload = encode_vehicle_state_estimate(_estimate(), leader_result.status)

        result = follower.step(_estimate(), (_message(1, 1, payload, 0.1),), now_monotonic=0.1)

        self.assertEqual(result.status.phase, FleetPhase.ACTIVE)
        self.assertEqual(result.controller_profile, "fleet_following")

class TestDistributedObserver(unittest.TestCase):
    def test_pass_through_observer_preserves_measurement_local_and_v2v_sources(self):
        observer = DistributedObserverFake({}, vehicle_id=2)
        observer.start()
        local_estimate = _estimate(1.0)
        measurement = VehicleStateEstimate(1.0, 9.0, 0.0, 0.0, 0.0, 0.0, True)
        peer_snapshot = FleetPeerSnapshot(
            source_vehicle_id=1,
            member_order=0,
            role=_member(1, "leader", 0).role,
            membership_revision=3,
            estimate=_estimate(1.0),
            source_sequence=4,
            source_timestamp=1.0,
            received_at_monotonic=1.1,
        )

        result = observer.update(
            local_estimate=local_estimate,
            peer_snapshots=(peer_snapshot,),
            membership_revision=3,
            measurements={2: measurement},
            dt=0.05,
        )

        self.assertEqual(result.observer_vehicle_id, 2)
        self.assertEqual(result.membership_revision, 3)
        self.assertEqual(result.source_for(1), DistributedEstimateSource.V2V)
        self.assertEqual(result.source_for(2), DistributedEstimateSource.MEASUREMENT)
        self.assertEqual(result.estimate_for(2), measurement)

    def test_manager_exposes_advisory_distributed_estimate_after_step(self):
        formation = FleetFormationBuilder().build(
            "distributed",
            (_member(1, "leader", 0), _member(2, "follower", 1)),
            _policy(),
        )
        manager = FleetManager(FleetRegistry(formation), vehicle_id=1)

        result = manager.step(_estimate(), (), now_monotonic=1.0, dt=0.05)

        self.assertIsNotNone(result.distributed_estimate)
        self.assertEqual(result.distributed_estimate.source_for(1), DistributedEstimateSource.LOCAL_OBSERVER)
        self.assertIs(result.distributed_estimate, manager.distributed_estimate())
        manager.shutdown()

    def test_luenberger_prototype_corrects_an_internal_prediction_without_controlling_fleet(self):
        observer = DistributedObserverLuenberger({"position_gain": 0.5}, vehicle_id=1)
        observer.start()
        observer.update(
            local_estimate=VehicleStateEstimate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True),
            peer_snapshots=(),
            membership_revision=0,
        )

        result = observer.update(
            local_estimate=VehicleStateEstimate(1.0, 10.0, 0.0, 0.0, 0.0, 0.0, True),
            peer_snapshots=(),
            membership_revision=0,
            dt=1.0,
        )

        estimate = result.estimate_for(1)
        self.assertIsNotNone(estimate)
        assert estimate is not None
        self.assertAlmostEqual(estimate.x, 5.0)
        self.assertEqual(result.source_for(1), DistributedEstimateSource.DISTRIBUTED_OBSERVER)


if __name__ == "__main__":
    unittest.main()
