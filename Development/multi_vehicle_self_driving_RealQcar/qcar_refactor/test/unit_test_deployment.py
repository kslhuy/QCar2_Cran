"""Local tests for the standalone physical-vehicle deployment CLI."""

import json
import os
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from extra.deployment.bundle import BundleError, build_bundle, load_bundle_spec, verify_bundle
from extra.deployment.cli import main
from extra.deployment.deployment_targets import (
    deploy_targets,
    load_deployment_targets,
    start_targets,
    stop_targets,
)
from extra.deployment.remote import SSHDeploymentClient
from extra.deployment.configuration import load_deployment_target
from extra.deployment.deployment_type import (
    BundleSummary,
    CommandResult,
    DeploymentError,
    DeploymentTarget,
    DeploymentTargetEntry,
    DeploymentTargets,
    PreflightReport,
)
from extra.platform.qcar.process_runner import QCarProcessManager, run_qcar_process
from core.vehicle_process import VehicleProcessSpec


class TestDeploymentBundle(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name) / "source"
        (self.root / "core").mkdir(parents=True)
        (self.root / "config").mkdir()
        (self.root / "test" / "artifacts").mkdir(parents=True)
        (self.root / "logs").mkdir()
        (self.root / ".git").mkdir()
        (self.root / "core" / "vehicle_main.py").write_text("print('drive')\n", encoding="utf-8")
        (self.root / "config" / "vehicle.yaml").write_text("vehicle: 0\n", encoding="utf-8")
        (self.root / "test" / "artifacts" / "result.csv").write_text("ignore\n", encoding="utf-8")
        (self.root / "logs" / "runtime.log").write_text("ignore\n", encoding="utf-8")
        (self.root / ".git" / "config").write_text("ignore\n", encoding="utf-8")

    def test_bundle_is_deterministic_and_excludes_generated_files(self):
        first = build_bundle(self.root, ["core", "config", "test", "logs"], self.root / "first.tar.gz")
        second = build_bundle(self.root, ["core", "config", "test", "logs"], self.root / "second.tar.gz")
        checked = verify_bundle(first.path)

        self.assertEqual(first.manifest_sha256, checked.manifest_sha256)
        self.assertEqual(first.file_count, 2)
        self.assertEqual(first.path.read_bytes(), second.path.read_bytes())

    def test_bundle_rejects_source_root_escape(self):
        with self.assertRaisesRegex(BundleError, "escapes source root"):
            build_bundle(self.root, ["../outside"], self.root / "bundle.tar.gz")

    def test_all_deployment_dataclasses_are_owned_by_the_type_module(self):
        for contract in (
            BundleSummary,
            CommandResult,
            DeploymentTarget,
            DeploymentTargetEntry,
            DeploymentTargets,
            PreflightReport,
        ):
            self.assertEqual(contract.__module__, "extra.deployment.deployment_type")

    def test_bundle_spec_resolves_source_root_relative_to_its_own_path(self):
        specs = self.root / "specs"
        specs.mkdir()
        spec = specs / "bundle.yaml"
        spec.write_text(
            "bundle:\n  source_root: ..\n  include:\n    - core\n    - config\n",
            encoding="utf-8",
        )
        source_root, includes = load_bundle_spec(spec)
        self.assertEqual(source_root, self.root.resolve())
        self.assertEqual(includes, ["core", "config"])

    def test_deployment_test_bundle_has_null_path_and_no_origin_route(self):
        project_root = Path(__file__).resolve().parents[1]
        spec_path = (
            project_root / "extra" / "deployment" / "config" / "bundles" / "qcar_deployment_test.bundle.yaml"
        )
        source_root, includes = load_bundle_spec(spec_path)
        bundle_path = self.root / "qcar-deployment-test.tar.gz"
        build_bundle(source_root, includes, bundle_path)
        summary = verify_bundle(bundle_path)
        self.assertGreater(summary.file_count, 0)

        with tempfile.TemporaryDirectory() as extraction:
            with tarfile.open(bundle_path, "r:gz") as archive:
                archive.extractall(extraction, filter="data")
            payload = Path(extraction) / "payload"
            self.assertFalse((payload / "refs" / "qcar_origin").exists())
            self.assertTrue((payload / "extra" / "platform" / "qcar" / "io_capture.py").is_file())
            sys.path.insert(0, str(payload))
            try:
                from core.vehicle_config import load_config

                config = load_config(
                    vehicle_config_file="config_vehicle_deployment_test.yaml"
                )
            finally:
                sys.path.remove(str(payload))
                sys.modules.pop("core.vehicle_config", None)
            self.assertIsNone(config.mission["path"])
            self.assertIsNone(config.module("planner").get("path_source"))
            self.assertEqual(config.module("ground_station")["server_host"], "192.168.1.135")


class TestDeploymentTargetAndCli(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        self.known_hosts = self.root / "known_hosts"
        self.known_hosts.write_text("qcar-0 ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIexample\n", encoding="utf-8")
        self.identity = self.root / "deploy_key"
        self.identity.write_text("not used in dry run\n", encoding="utf-8")
        self.target_file = self.root / "target.yaml"
        self.target_file.write_text(
            yaml.safe_dump(
                {
                    "deployment_target": {
                        "vehicle_id": 0,
                        "ssh": {
                            "host": "192.168.1.230",
                            "username": "nvidia",
                            "known_hosts": str(self.known_hosts),
                            "identity_file": str(self.identity),
                        },
                        "release": {"root": "/home/nvidia/qcar-deployments"},
                    }
                }
            ),
            encoding="utf-8",
        )

    def test_target_rejects_embedded_password(self):
        contents = yaml.safe_load(self.target_file.read_text(encoding="utf-8"))
        contents["deployment_target"]["ssh"]["password"] = "not-allowed"
        self.target_file.write_text(yaml.safe_dump(contents), encoding="utf-8")
        with self.assertRaisesRegex(DeploymentError, "must not contain a password"):
            load_deployment_target(self.target_file)

    def test_target_rejects_the_removed_flat_schema(self):
        self.target_file.write_text(
            yaml.safe_dump(
                {
                    "target": {
                        "vehicle_id": 0,
                        "host": "192.168.1.230",
                        "username": "nvidia",
                        "remote_root": "/home/nvidia/qcar-deployments",
                        "known_hosts": str(self.known_hosts),
                    }
                }
            ),
            encoding="utf-8",
        )

        with self.assertRaisesRegex(DeploymentError, "unsupported fields: target"):
            load_deployment_target(self.target_file)

    def test_target_allows_a_direct_password_prompt_without_storing_a_password(self):
        contents = yaml.safe_load(self.target_file.read_text(encoding="utf-8"))
        contents["deployment_target"]["ssh"].pop("identity_file")
        contents["deployment_target"]["ssh"]["password_prompt"] = True
        self.target_file.write_text(yaml.safe_dump(contents), encoding="utf-8")
        target = load_deployment_target(self.target_file)
        self.assertTrue(target.password_prompt)
        self.assertIsNone(target.password_env)

    def test_dry_run_requires_matching_explicit_target_and_confirmation(self):
        bundle = self.root / "bundle.tar.gz"
        source = self.root / "source"
        source.mkdir()
        (source / "main.py").write_text("print('ok')\n", encoding="utf-8")
        build_bundle(source, ["main.py"], bundle)

        success = main(
            [
                "deploy", "--target-file", str(self.target_file), "--vehicle-id", "0",
                "--host", "192.168.1.230", "--bundle", str(bundle), "--yes", "--dry-run",
            ]
        )
        missing_confirmation = main(
            [
                "deploy", "--target-file", str(self.target_file), "--vehicle-id", "0",
                "--host", "192.168.1.230", "--bundle", str(bundle), "--dry-run",
            ]
        )
        self.assertEqual(success, 0)
        self.assertEqual(missing_confirmation, 2)

    def test_rollback_dry_run_requires_a_release_id_and_explicit_target(self):
        success = main(
            [
                "rollback", "--target-file", str(self.target_file), "--vehicle-id", "0",
                "--host", "192.168.1.230", "--release", "release-0123456789abcdef",
                "--yes", "--dry-run",
            ]
        )
        self.assertEqual(success, 0)

    def test_preflight_reconnects_once_and_reuses_the_same_password(self):
        target = load_deployment_target(self.target_file)
        client = SSHDeploymentClient(target)
        first_error = DeploymentError("first session dropped")
        report = object()
        with patch.object(client, "_run_preflight_checks", side_effect=[first_error, report]) as checks, patch.object(
            client, "close"
        ) as close, patch.object(client, "connect") as connect:
            self.assertIs(client.preflight(), report)
        self.assertEqual(checks.call_count, 2)
        close.assert_called_once()
        connect.assert_called_once()

    def test_deploy_reconnects_once_when_no_remote_command_could_start(self):
        target = load_deployment_target(self.target_file)
        client = SSHDeploymentClient(target)
        source = self.root / "retry-source"
        source.mkdir()
        (source / "main.py").write_text("print('retry')\n", encoding="utf-8")
        bundle = build_bundle(source, ["main.py"], self.root / "retry.tar.gz")
        first_error = DeploymentError("Remote command failed to run: SSH session not active")
        with patch.object(
            client,
            "_deploy_bundle_once",
            side_effect=[first_error, "release-0123456789abcdef"],
        ) as deploy_once, patch.object(
            client, "close"
        ) as close, patch.object(client, "connect") as connect:
            self.assertEqual(client.deploy_bundle(bundle.path), "release-0123456789abcdef")
        self.assertEqual(deploy_once.call_count, 2)
        close.assert_called_once()
        connect.assert_called_once()

    def test_fetch_artifact_reports_a_missing_remote_path_without_traceback(self):
        target = load_deployment_target(self.target_file)
        client = SSHDeploymentClient(target)

        class MissingArtifactSFTP:
            closed = False

            def stat(self, _path):
                raise FileNotFoundError("No such file")

            def close(self):
                self.closed = True

        class FakeSSHClient:
            def __init__(self, sftp):
                self._sftp = sftp

            def open_sftp(self):
                return self._sftp

        sftp = MissingArtifactSFTP()
        client._client = FakeSSHClient(sftp)
        with self.assertRaisesRegex(DeploymentError, "Remote artifact does not exist"):
            client.fetch_artifact("artifacts/missing", self.root / "download")
        self.assertTrue(sftp.closed)


class TestDeploymentTargets(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        self.source = self.root / "source"
        self.source.mkdir()
        (self.source / "main.py").write_text("print('fleet')\n", encoding="utf-8")
        self.spec = self.root / "bundle.yaml"
        self.spec.write_text(
            "bundle:\n  source_root: source\n  include:\n    - main.py\n",
            encoding="utf-8",
        )
        self.known_hosts = self.root / "known_hosts"
        self.known_hosts.write_text(
            "qcar-0 ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIexample\n", encoding="utf-8"
        )
        self.identity = self.root / "deploy_key"
        self.identity.write_text("not used in unit tests\n", encoding="utf-8")
        self.target = self.root / "target.yaml"
        self.target.write_text(
            yaml.safe_dump(
                {
                    "deployment_target": {
                        "vehicle_id": 0,
                        "ssh": {
                            "host": "192.168.1.230",
                            "username": "nvidia",
                            "known_hosts": str(self.known_hosts),
                            "identity_file": str(self.identity),
                        },
                        "release": {"root": "/home/nvidia/qcar-deployments"},
                        "expected_hostname": "qcar-0",
                    }
                }
            ),
            encoding="utf-8",
        )
        self.targets_file = self.root / "deployment_targets.yaml"
        self.targets_file.write_text(
            yaml.safe_dump(
                {
                    "deployment_targets": {
                        "bundle": {
                            "specification": str(self.spec),
                            "output": str(self.root / "targets.tar.gz"),
                        },
                        "targets": [{"file": str(self.target)}],
                    }
                }
            ),
            encoding="utf-8",
        )

    def test_targets_dry_run_validates_the_local_plan_without_connecting(self):
        for command in ("targets-deploy", "targets-start", "targets-stop"):
            result = main([command, "--targets-file", str(self.targets_file), "--yes", "--dry-run"])
            self.assertEqual(result, 0)
        self.assertFalse((self.root / "targets.tar.gz").exists())

    def test_targets_cli_rejects_the_removed_fleet_alias(self):
        with self.assertRaises(SystemExit) as error:
            main(["fleet-deploy", "--fleet-file", str(self.targets_file), "--yes", "--dry-run"])

        self.assertEqual(error.exception.code, 2)

    def test_targets_inventory_rejects_removed_legacy_shape(self):
        self.targets_file.write_text(
            yaml.safe_dump(
                {
                    "fleet": {
                        "bundle_spec": str(self.spec),
                        "bundle_output": str(self.root / "targets.tar.gz"),
                        "vehicles": [{"target_file": str(self.target)}],
                    }
                }
            ),
            encoding="utf-8",
        )

        with self.assertRaisesRegex(DeploymentError, "unsupported fields: fleet"):
            load_deployment_targets(self.targets_file)

    def test_targets_deploy_packages_once_after_all_preflights_pass(self):
        calls = []
        progress = []

        class FakeClient:
            def __init__(self, target):
                self.target = target

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return None

            def preflight(self):
                calls.append(("preflight", self.target.vehicle_id))
                return PreflightReport(True, "qcar-0", "aarch64", "/dev/root", (), ())

            def deploy_bundle(self, bundle):
                calls.append(("deploy", Path(bundle).name))
                return "release-0123456789abcdef"

        result = deploy_targets(
            load_deployment_targets(self.targets_file),
            client_factory=FakeClient,
            progress=lambda label, completed, total: progress.append((label, completed, total)),
        )
        self.assertEqual(result["releases"][0]["release"], "release-0123456789abcdef")
        self.assertEqual(calls, [("preflight", 0), ("deploy", "targets.tar.gz")])
        self.assertEqual(
            progress,
            [
                ("Packaging selected bundle", 0, 3),
                ("Bundle verified", 1, 3),
                ("Preflight vehicle 0", 2, 3),
                ("Deployed vehicle 0", 3, 3),
            ],
        )
        self.assertTrue((self.root / "targets.tar.gz").is_file())

    def test_targets_start_and_stop_return_common_endpoint_reports(self):
        calls = []

        class FakeClient:
            def __init__(self, target):
                self.target = target

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return None

            def preflight(self):
                calls.append(("preflight", self.target.vehicle_id))
                return PreflightReport(True, "qcar-0", "aarch64", "/dev/root", (), ())

            def start(self):
                calls.append(("start", self.target.vehicle_id))
                return CommandResult("start", 0, "1234", "")

            def stop(self):
                calls.append(("stop", self.target.vehicle_id))
                return CommandResult("stop", 0, "", "")

            def tail_logs(self, _lines):
                return CommandResult("logs", 0, "READY", "")

        targets = load_deployment_targets(self.targets_file)
        started = start_targets(targets, client_factory=FakeClient)
        stopped = stop_targets(targets, client_factory=FakeClient)

        self.assertTrue(started.ok)
        self.assertEqual(started.endpoints[0].logs, "READY")
        self.assertEqual(started.endpoints[0].shutdown, "not_requested")
        self.assertTrue(stopped.ok)
        self.assertEqual(stopped.endpoints[0].shutdown, "graceful")
        self.assertEqual(calls, [("preflight", 0), ("start", 0), ("stop", 0)])


class TestQCarRuntimeBootstrap(unittest.TestCase):
    def test_manager_resolves_implicit_vehicle_id_from_its_config(self):
        context = QCarProcessManager(
            VehicleProcessSpec(None, "config_vehicle_deployment_test.yaml")
        ).prepare()

        self.assertEqual(context.vehicle.vehicle_id, 0)
        self.assertEqual(context.spec.vehicle_config_file, "config_vehicle_deployment_test.yaml")

    def test_runtime_stops_qcar_and_bootstrap_delegates_final_neutral_to_pal(self):
        class FakeQCar:
            def __init__(self):
                self.writes = []
                self.writes_before_terminate = None
                self.terminated = False

            def write(self, *, throttle, steering):
                self.writes.append((throttle, steering))

            def terminate(self):
                self.writes_before_terminate = len(self.writes)
                self.write(throttle=0.0, steering=0.0)
                self.terminated = True

        class FakeRuntime:
            class Config:
                runtime = {"loop_rate_hz": 100}

            config = Config()

            def __init__(self, qcar):
                self._qcar = qcar
                self.started = False
                self.steps = 0
                self.stopped = False

            def start(self):
                self.started = True

            def step(self, dt=None):
                self.steps += 1

            def shutdown(self):
                # This represents IOQCar2.stop(), invoked by VehicleRuntime.shutdown().
                self._qcar.write(throttle=0.0, steering=0.0)
                self.stopped = True

        qcar = FakeQCar()
        runtime = FakeRuntime(qcar)
        received_specs = []
        sleeps = []

        def factory(**kwargs):
            self.assertEqual(kwargs, {"readMode": 1})
            return qcar

        def builder(spec):
            received_specs.append(spec)
            return runtime

        run_qcar_process(
            VehicleProcessSpec(0, "config_vehicle_deployment_test.yaml"),
            cycles=2,
            qcar_factory=factory,
            runtime_builder=builder,
            sleep=sleeps.append,
            monotonic=lambda: 1.0,
        )

        self.assertIs(received_specs[0].resources["qcar"], qcar)
        self.assertTrue(runtime.started)
        self.assertEqual(runtime.steps, 2)
        self.assertTrue(runtime.stopped)
        self.assertTrue(qcar.terminated)
        self.assertEqual(qcar.writes_before_terminate, 1)
        self.assertEqual(qcar.writes, [(0.0, 0.0), (0.0, 0.0)])
        self.assertEqual(sleeps, [0.01, 0.01])


if __name__ == "__main__":
    unittest.main()
