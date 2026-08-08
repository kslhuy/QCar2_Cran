"""Pinned-host-key SSH operations for the standalone deployment CLI."""

from __future__ import annotations

import posixpath
import re
import shlex
import stat
import uuid
from pathlib import Path
from typing import Any

from .bundle import verify_bundle
from .deployment_type import (
    BundleSummary,
    CommandResult,
    DeploymentError,
    DeploymentTarget,
    PreflightReport,
)


_REMOTE_MANIFEST_VERIFY = """
import hashlib
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
manifest = json.loads((root / 'manifest.json').read_text(encoding='utf-8'))
for entry in manifest['files']:
    relative = Path(entry['path'])
    if relative.is_absolute() or '..' in relative.parts:
        raise RuntimeError('unsafe manifest path: ' + entry['path'])
    payload = root / 'payload' / relative
    if payload.is_symlink() or not payload.is_file():
        raise RuntimeError('missing payload: ' + entry['path'])
    digest = hashlib.sha256(payload.read_bytes()).hexdigest()
    if digest != entry['sha256'] or payload.stat().st_size != entry['size']:
        raise RuntimeError('manifest mismatch: ' + entry['path'])
print('manifest verified: ' + str(len(manifest['files'])) + ' files')
""".strip()


class SSHDeploymentClient:
    """Small SSH/SFTP adapter that rejects unknown host keys by default."""

    def __init__(self, target: DeploymentTarget):
        self.target = target
        self._client: Any | None = None
        self._password: str | None = None

    def __enter__(self) -> "SSHDeploymentClient":
        self.connect()
        return self

    def __exit__(self, exception_type, exception, traceback) -> None:
        self.close()

    def connect(self) -> None:
        if self._client is not None:
            return
        if not self.target.known_hosts.is_file():
            raise DeploymentError(
                f"Pinned known_hosts file does not exist: {self.target.known_hosts}"
            )
        if self.target.identity_file is not None and not self.target.identity_file.is_file():
            raise DeploymentError(f"SSH identity file does not exist: {self.target.identity_file}")
        try:
            import paramiko
        except ImportError as error:
            raise DeploymentError("paramiko is required for remote deployment operations") from error

        client = paramiko.SSHClient()
        client.load_host_keys(str(self.target.known_hosts))
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        if self._password is None:
            self._password = self.target.password()
        try:
            client.connect(
                hostname=self.target.host,
                port=self.target.port,
                username=self.target.username,
                key_filename=str(self.target.identity_file) if self.target.identity_file else None,
                password=self._password,
                timeout=self.target.connect_timeout_s,
                banner_timeout=self.target.connect_timeout_s,
                auth_timeout=self.target.connect_timeout_s,
                allow_agent=True,
                look_for_keys=self.target.identity_file is None,
            )
        except Exception as error:
            client.close()
            raise DeploymentError(f"SSH connection to {self.target.host} failed: {error}") from error
        self._client = client

    def close(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    def run(self, command: str, *, check: bool = True) -> CommandResult:
        if self._client is None:
            raise DeploymentError("SSH client is not connected")
        try:
            _, stdout, stderr = self._client.exec_command(
                command, timeout=self.target.command_timeout_s
            )
            exit_status = stdout.channel.recv_exit_status()
            result = CommandResult(
                command=command,
                exit_status=exit_status,
                stdout=stdout.read().decode("utf-8", errors="replace").strip(),
                stderr=stderr.read().decode("utf-8", errors="replace").strip(),
            )
        except Exception as error:
            raise DeploymentError(f"Remote command failed to run: {error}") from error
        if check and result.exit_status != 0:
            detail = result.stderr or result.stdout or "no remote output"
            raise DeploymentError(
                f"Remote command failed ({result.exit_status}): {detail}"
            )
        return result

    def _run_preflight_checks(self) -> PreflightReport:
        hostname = self.run("hostname").stdout
        architecture = self.run("uname -m").stdout
        parent = posixpath.dirname(self.target.remote_root)
        free_space = self.run(f"df -Pk {shlex.quote(parent)} | tail -n 1").stdout
        missing = []
        for command in self.target.required_commands:
            result = self.run(f"command -v {shlex.quote(command)}", check=False)
            if result.exit_status != 0:
                missing.append(command)
        missing_python_modules = []
        module_probe = (
            "import importlib.util, sys; "
            "sys.exit(0 if importlib.util.find_spec(sys.argv[1]) is not None else 1)"
        )
        for module in self.target.required_python_modules:
            result = self.run(
                f"python3 -c {shlex.quote(module_probe)} {shlex.quote(module)}", check=False
            )
            if result.exit_status != 0:
                missing_python_modules.append(module)
        hostname_matches = (
            self.target.expected_hostname is None
            or hostname == self.target.expected_hostname
        )
        return PreflightReport(
            ok=hostname_matches and not missing and not missing_python_modules,
            hostname=hostname,
            architecture=architecture,
            free_space=free_space,
            missing_commands=tuple(missing),
            missing_python_modules=tuple(missing_python_modules),
        )

    def preflight(self) -> PreflightReport:
        """Run read-only checks, reconnecting once if the SSH transport drops."""

        try:
            return self._run_preflight_checks()
        except DeploymentError as first_error:
            self.close()
            try:
                self.connect()
                return self._run_preflight_checks()
            except DeploymentError as retry_error:
                raise DeploymentError(
                    "Remote preflight failed after reconnecting once: "
                    f"first error: {first_error}; retry error: {retry_error}"
                ) from retry_error

    def _require_passing_preflight(self, *, require_identity: bool = False) -> PreflightReport:
        report = self.preflight()
        if not report.ok:
            problems = []
            if self.target.expected_hostname and report.hostname != self.target.expected_hostname:
                problems.append(
                    f"expected hostname {self.target.expected_hostname!r}, got {report.hostname!r}"
                )
            if report.missing_commands:
                problems.append("missing commands: " + ", ".join(report.missing_commands))
            if report.missing_python_modules:
                problems.append(
                    "missing Python modules: " + ", ".join(report.missing_python_modules)
                )
            raise DeploymentError("Remote preflight failed: " + "; ".join(problems))
        if require_identity and not self.target.expected_hostname:
            raise DeploymentError(
                "Mutating deployment operations require target.expected_hostname "
                "to verify vehicle identity"
            )
        return report

    def deploy_bundle(self, bundle_path: str | Path) -> str:
        """Verify, stage, verify remotely, and atomically activate a release.

        A dropped SSH session means the command could not be started, so one
        reconnect-and-retry is safe. Each attempt uses a new staging directory;
        an already-created release is re-verified before activation.
        """

        bundle: BundleSummary = verify_bundle(bundle_path)
        try:
            return self._deploy_bundle_once(bundle)
        except DeploymentError as first_error:
            if not self._is_transport_error(first_error):
                raise
            self.close()
            try:
                self.connect()
                return self._deploy_bundle_once(bundle)
            except DeploymentError as retry_error:
                raise DeploymentError(
                    "Remote deployment failed after reconnecting once: "
                    f"first error: {first_error}; retry error: {retry_error}"
                ) from retry_error

    @staticmethod
    def _is_transport_error(error: DeploymentError) -> bool:
        """Return whether no remote command was started because SSH was inactive."""

        return str(error).startswith("Remote command failed to run:")

    def _deploy_bundle_once(self, bundle: BundleSummary) -> str:
        """Perform one transactional deployment attempt over an active SSH client."""

        self._require_passing_preflight(require_identity=True)
        release_name = f"release-{bundle.manifest_sha256[:16]}"
        release_path = posixpath.join(self.target.remote_root, "releases", release_name)
        staging_path = posixpath.join(
            self.target.remote_root, ".staging", f"{release_name}-{uuid.uuid4().hex}"
        )
        quoted = shlex.quote
        self.run(
            f"mkdir -p {quoted(posixpath.join(self.target.remote_root, '.staging'))} "
            f"{quoted(posixpath.join(self.target.remote_root, 'releases'))} "
            f"{quoted(posixpath.join(self.target.remote_root, 'logs'))}"
        )
        try:
            self.run(f"mkdir {quoted(staging_path)}")
            if self.run(f"test -e {quoted(release_path)}", check=False).exit_status == 0:
                self.run(
                    f"python3 -c {quoted(_REMOTE_MANIFEST_VERIFY)} {quoted(release_path)}"
                )
            else:
                sftp = self._client.open_sftp()
                try:
                    sftp.put(str(bundle.path), posixpath.join(staging_path, "bundle.tar.gz"))
                finally:
                    sftp.close()
                self.run(
                    f"tar --no-same-owner --no-same-permissions -xzf "
                    f"{quoted(posixpath.join(staging_path, 'bundle.tar.gz'))} "
                    f"-C {quoted(staging_path)}"
                )
                self.run(f"rm -f {quoted(posixpath.join(staging_path, 'bundle.tar.gz'))}")
                self.run(
                    f"python3 -c {quoted(_REMOTE_MANIFEST_VERIFY)} {quoted(staging_path)}"
                )
                self.run(f"mv {quoted(staging_path)} {quoted(release_path)}")
            self.run(
                f"ln -sfn {quoted(posixpath.join('releases', release_name))} "
                f"{quoted(posixpath.join(self.target.remote_root, 'current.next'))} && "
                f"mv -Tf {quoted(posixpath.join(self.target.remote_root, 'current.next'))} "
                f"{quoted(posixpath.join(self.target.remote_root, 'current'))}"
            )
        except Exception:
            self.run(f"rm -rf {quoted(staging_path)}", check=False)
            raise
        self.run(f"rm -rf {quoted(staging_path)}", check=False)
        return release_name

    def rollback(self, release_name: str) -> str:
        """Verify and atomically reactivate a previously uploaded release."""

        if not re.fullmatch(r"release-[0-9a-f]{16}", release_name):
            raise DeploymentError("Rollback release name must be a deployment release ID")
        self._require_passing_preflight(require_identity=True)
        release_path = posixpath.join(self.target.remote_root, "releases", release_name)
        quoted = shlex.quote
        self.run(f"test -d {quoted(release_path)}")
        self.run(f"python3 -c {quoted(_REMOTE_MANIFEST_VERIFY)} {quoted(release_path)}")
        self.run(
            f"ln -sfn {quoted(posixpath.join('releases', release_name))} "
            f"{quoted(posixpath.join(self.target.remote_root, 'current.next'))} && "
            f"mv -Tf {quoted(posixpath.join(self.target.remote_root, 'current.next'))} "
            f"{quoted(posixpath.join(self.target.remote_root, 'current'))}"
        )
        return release_name

    def start(self) -> CommandResult:
        if not self.target.start_command:
            raise DeploymentError("deployment.start_command is required for start")
        self._require_passing_preflight(require_identity=True)
        release_path = posixpath.join(self.target.remote_root, "current")
        log_path = self.target.render(self.target.log_path_template, release=release_path)
        command = self.target.render(self.target.start_command, release=release_path)
        pid_path = posixpath.join(
            self.target.remote_root, "run", f"vehicle-{self.target.vehicle_id}.pid"
        )
        script = (
            "set -e; "
            f"test -d {shlex.quote(posixpath.join(release_path, 'payload'))}; "
            f"mkdir -p {shlex.quote(posixpath.dirname(log_path))} {shlex.quote(posixpath.dirname(pid_path))}; "
            f"if test -f {shlex.quote(pid_path)}; then "
            f"existing=$(cat {shlex.quote(pid_path)}); "
            "case \"$existing\" in ''|*[!0-9]*) ;; "
            "*) if kill -0 \"$existing\" 2>/dev/null; then "
            "echo \"vehicle process already running: $existing\" >&2; exit 3; fi ;; esac; "
            f"rm -f {shlex.quote(pid_path)}; fi; "
            f"cd {shlex.quote(posixpath.join(release_path, 'payload'))}; "
            f"nohup {command} > {shlex.quote(log_path)} 2>&1 & "
            "pid=$!; "
            f"echo \"$pid\" > {shlex.quote(pid_path)}; "
            "sleep 2; kill -0 \"$pid\"; echo \"$pid\""
        )
        return self.run(f"sh -c {shlex.quote(script)}")

    def stop(self) -> CommandResult:
        if not self.target.stop_command:
            raise DeploymentError("deployment.stop_command is required for stop")
        self._require_passing_preflight(require_identity=True)
        return self.run(self.target.render(self.target.stop_command), check=False)

    def tail_logs(self, lines: int) -> CommandResult:
        if not 1 <= lines <= 1000:
            raise DeploymentError("Log tail line count must be between 1 and 1000")
        log_path = self.target.render(self.target.log_path_template)
        return self.run(f"tail -n {lines} {shlex.quote(log_path)}", check=False)

    def fetch_artifact(self, relative_path: str, output_directory: str | Path) -> Path:
        relative = Path(relative_path)
        if relative.is_absolute() or ".." in relative.parts:
            raise DeploymentError("Artifact path must be relative to the active release")
        remote_path = posixpath.join(self.target.remote_root, "current", relative.as_posix())
        destination = Path(output_directory).resolve() / relative
        if self._client is None:
            raise DeploymentError("SSH client is not connected")
        sftp = self._client.open_sftp()
        try:
            try:
                self._download_path(sftp, remote_path, destination)
            except OSError as error:
                raise DeploymentError(
                    f"Remote artifact does not exist or cannot be read: {relative_path}"
                ) from error
        finally:
            sftp.close()
        return destination

    def _download_path(self, sftp: Any, remote_path: str, destination: Path) -> None:
        attributes = sftp.stat(remote_path)
        if stat.S_ISDIR(attributes.st_mode):
            destination.mkdir(parents=True, exist_ok=True)
            for entry in sftp.listdir_attr(remote_path):
                if entry.filename in {".", ".."}:
                    continue
                self._download_path(
                    sftp,
                    posixpath.join(remote_path, entry.filename),
                    destination / entry.filename,
                )
            return
        destination.parent.mkdir(parents=True, exist_ok=True)
        sftp.get(remote_path, str(destination))
