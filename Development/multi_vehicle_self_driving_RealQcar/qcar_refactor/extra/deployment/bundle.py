"""Deterministic, allowlisted deployment bundle creation and verification."""

from __future__ import annotations

from hashlib import sha256
from io import BytesIO
import gzip
import json
from pathlib import Path
import tarfile
from typing import Iterable

import yaml

from .deployment_type import BundleSummary


class BundleError(ValueError):
    """Raised when a deployment bundle would be incomplete or unsafe."""


_EXCLUDED_DIRECTORY_NAMES = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    "__pycache__",
    "build",
    "install",
    "log",
    "logs",
    "node_modules",
}
_EXCLUDED_SUFFIXES = {".log", ".pyc", ".pyo"}
_EXCLUDED_PATH_PREFIXES = (("test", "artifacts"), ("scope_recordings",))


def load_bundle_spec(spec_path: str | Path) -> tuple[Path, list[str]]:
    """Load an explicit, versioned bundle selection file.

    ``source_root`` is resolved relative to the specification, making the same
    spec usable from any current working directory.  The specification contains
    source-selection data only; target hosts and credentials belong in ignored
    local target files.
    """

    path = Path(spec_path).resolve()
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except OSError as error:
        raise BundleError(f"Cannot read bundle specification {path}: {error}") from error
    except yaml.YAMLError as error:
        raise BundleError(f"Invalid YAML in bundle specification {path}: {error}") from error
    if not isinstance(raw, dict) or not isinstance(raw.get("bundle"), dict):
        raise BundleError("Bundle specification must contain a 'bundle' mapping")
    specification = raw["bundle"]
    source_root = specification.get("source_root")
    includes = specification.get("include")
    if not isinstance(source_root, str) or not source_root.strip():
        raise BundleError("Bundle specification requires a non-empty source_root")
    if not isinstance(includes, list) or not includes or not all(
        isinstance(item, str) and item.strip() for item in includes
    ):
        raise BundleError("Bundle specification requires a non-empty include list")
    return (path.parent / source_root).resolve(), [item.strip() for item in includes]


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _normalise_member_path(path: Path) -> str:
    value = path.as_posix()
    if not value or value.startswith("/") or ".." in path.parts:
        raise BundleError(f"Unsafe bundle member path: {value!r}")
    return value


def _is_excluded(relative_path: Path) -> bool:
    parts = relative_path.parts
    if any(part in _EXCLUDED_DIRECTORY_NAMES for part in parts[:-1]):
        return True
    if any(parts[: len(prefix)] == prefix for prefix in _EXCLUDED_PATH_PREFIXES):
        return True
    name = relative_path.name
    return name.startswith(".env") or relative_path.suffix.lower() in _EXCLUDED_SUFFIXES


def _selected_files(source_root: Path, includes: Iterable[str]) -> list[tuple[Path, Path]]:
    files: dict[Path, Path] = {}
    for include in includes:
        candidate = (source_root / include).resolve()
        if not _is_relative_to(candidate, source_root):
            raise BundleError(f"Included path escapes source root: {include!r}")
        if not candidate.exists():
            raise BundleError(f"Included path does not exist: {include!r}")
        if candidate.is_symlink():
            raise BundleError(f"Symlinks are not allowed in a deployment bundle: {include!r}")

        if candidate.is_file():
            relative_path = candidate.relative_to(source_root)
            if not _is_excluded(relative_path):
                files[relative_path] = candidate
            continue

        for child in sorted(candidate.rglob("*")):
            if child.is_symlink():
                raise BundleError(
                    f"Symlinks are not allowed in a deployment bundle: "
                    f"{child.relative_to(source_root).as_posix()}"
                )
            if not child.is_file():
                continue
            relative_path = child.relative_to(source_root)
            if not _is_excluded(relative_path):
                files[relative_path] = child

    if not files:
        raise BundleError("The selected deployment bundle contains no deployable files")
    return sorted(files.items(), key=lambda item: item[0].as_posix())


def _manifest_bytes(entries: list[dict[str, object]]) -> bytes:
    manifest = {"schema_version": 1, "files": entries}
    return (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _tar_info(name: str, size: int, mode: int = 0o644) -> tarfile.TarInfo:
    info = tarfile.TarInfo(name)
    info.size = size
    info.mode = mode
    info.mtime = 0
    info.uid = 0
    info.gid = 0
    info.uname = "root"
    info.gname = "root"
    return info


def build_bundle(source_root: str | Path, includes: Iterable[str], output_path: str | Path) -> BundleSummary:
    """Create a repeatable tar.gz archive from explicitly selected source paths.

    Paths are evaluated relative to ``source_root``.  Common generated output,
    secrets in ``.env`` files, logs, VCS metadata, and symlinks are rejected or
    excluded so that a successful bundle has a clear, reproducible manifest.
    """

    root = Path(source_root).resolve()
    if not root.is_dir():
        raise BundleError(f"Source root is not a directory: {root}")

    selected = _selected_files(root, includes)
    entries: list[dict[str, object]] = []
    contents: list[tuple[str, bytes, int]] = []
    for relative_path, local_path in selected:
        data = local_path.read_bytes()
        member_path = _normalise_member_path(relative_path)
        mode = 0o755 if local_path.stat().st_mode & 0o111 else 0o644
        entries.append(
            {
                "path": member_path,
                "sha256": sha256(data).hexdigest(),
                "size": len(data),
            }
        )
        contents.append((member_path, data, mode))

    manifest_bytes = _manifest_bytes(entries)
    destination = Path(output_path).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("wb") as raw_file:
        with gzip.GzipFile(filename="", fileobj=raw_file, mode="wb", mtime=0) as gzip_file:
            with tarfile.open(fileobj=gzip_file, mode="w") as archive:
                archive.addfile(_tar_info("manifest.json", len(manifest_bytes)), BytesIO(manifest_bytes))
                for member_path, data, mode in contents:
                    archive.addfile(
                        _tar_info(f"payload/{member_path}", len(data), mode),
                        BytesIO(data),
                    )

    return BundleSummary(
        path=destination,
        manifest_sha256=sha256(manifest_bytes).hexdigest(),
        file_count=len(entries),
        total_bytes=sum(int(entry["size"]) for entry in entries),
    )


def verify_bundle(bundle_path: str | Path) -> BundleSummary:
    """Verify every manifest entry in a locally generated deployment archive."""

    archive_path = Path(bundle_path).resolve()
    try:
        with tarfile.open(archive_path, mode="r:gz") as archive:
            members = archive.getmembers()
            names = [member.name for member in members]
            if len(names) != len(set(names)) or "manifest.json" not in names:
                raise BundleError("Bundle must contain one manifest.json and no duplicate members")
            if any(member.issym() or member.islnk() or not member.isfile() for member in members):
                raise BundleError("Bundle contains a non-regular file")
            if any(name.startswith("/") or ".." in Path(name).parts for name in names):
                raise BundleError("Bundle contains an unsafe member path")

            manifest_file = archive.extractfile("manifest.json")
            if manifest_file is None:
                raise BundleError("Bundle manifest cannot be read")
            manifest_bytes = manifest_file.read()
            try:
                manifest = json.loads(manifest_bytes.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                raise BundleError(f"Bundle manifest is invalid JSON: {error}") from error

            entries = manifest.get("files")
            if manifest.get("schema_version") != 1 or not isinstance(entries, list):
                raise BundleError("Bundle manifest schema is not supported")
            expected_payload_names = set()
            total_bytes = 0
            for entry in entries:
                if not isinstance(entry, dict):
                    raise BundleError("Bundle manifest contains an invalid file entry")
                path = entry.get("path")
                digest = entry.get("sha256")
                size = entry.get("size")
                if not isinstance(path, str) or not isinstance(digest, str) or not isinstance(size, int):
                    raise BundleError("Bundle manifest file entry is incomplete")
                member_name = f"payload/{_normalise_member_path(Path(path))}"
                if member_name in expected_payload_names:
                    raise BundleError(f"Bundle manifest repeats a path: {path}")
                expected_payload_names.add(member_name)
                source = archive.extractfile(member_name)
                if source is None:
                    raise BundleError(f"Bundle is missing payload file: {path}")
                data = source.read()
                if len(data) != size or sha256(data).hexdigest() != digest:
                    raise BundleError(f"Bundle hash mismatch: {path}")
                total_bytes += size

            if set(names) != expected_payload_names | {"manifest.json"}:
                raise BundleError("Bundle contains files that are not declared in its manifest")
    except (OSError, tarfile.TarError) as error:
        raise BundleError(f"Cannot read deployment bundle {archive_path}: {error}") from error

    return BundleSummary(
        path=archive_path,
        manifest_sha256=sha256(manifest_bytes).hexdigest(),
        file_count=len(entries),
        total_bytes=total_bytes,
    )
