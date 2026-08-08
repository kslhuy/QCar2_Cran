"""Standard test-artifact run layout and manifest creation.

This module is deliberately test-only.  It gives every durable test result a
unique run directory without affecting vehicle, simulator, or deployment
runtime behavior.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
from typing import Any, Mapping
from uuid import uuid4


_DEFAULT_ROOT = Path(__file__).resolve().parent / "artifacts"
_IDENTIFIER = re.compile(r"[a-z0-9][a-z0-9_-]*")


@dataclass(frozen=True)
class ArtifactRun:
    """One immutable directory layout for a generated test result."""

    category: str
    platform: str
    test_name: str
    run_id: str
    directory: Path
    raw_directory: Path
    derived_directory: Path
    figures_directory: Path
    logs_directory: Path
    manifest_path: Path


def create_artifact_run(
    *,
    category: str,
    platform: str,
    test_name: str,
    root: str | Path | None = None,
    metadata: Mapping[str, Any] | None = None,
    run_id: str | None = None,
) -> ArtifactRun:
    """Create a unique standard artifact layout and its JSON manifest.

    Layout::

        <root>/<category>/<platform>/<test_name>/<run_id>/
            manifest.json
            raw/
            derived/
            figures/
            logs/
    """

    category = _identifier(category, "category")
    platform = _identifier(platform, "platform")
    test_name = _identifier(test_name, "test_name")
    selected_run_id = _identifier(run_id or _new_run_id(), "run_id")
    base = Path(root) if root is not None else _DEFAULT_ROOT
    directory = base / category / platform / test_name / selected_run_id
    if directory.exists():
        raise FileExistsError(f"Artifact run directory already exists: {directory}")

    raw_directory = directory / "raw"
    derived_directory = directory / "derived"
    figures_directory = directory / "figures"
    logs_directory = directory / "logs"
    for path in (raw_directory, derived_directory, figures_directory, logs_directory):
        path.mkdir(parents=True, exist_ok=False)

    manifest_path = directory / "manifest.json"
    manifest = {
        "schema_version": 1,
        "category": category,
        "platform": platform,
        "test_name": test_name,
        "run_id": selected_run_id,
        "created_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "layout": {
            "raw": "raw",
            "derived": "derived",
            "figures": "figures",
            "logs": "logs",
        },
        "metadata": dict(metadata or {}),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return ArtifactRun(
        category=category,
        platform=platform,
        test_name=test_name,
        run_id=selected_run_id,
        directory=directory,
        raw_directory=raw_directory,
        derived_directory=derived_directory,
        figures_directory=figures_directory,
        logs_directory=logs_directory,
        manifest_path=manifest_path,
    )


def _identifier(value: str, name: str) -> str:
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ValueError(f"Artifact {name} must use lowercase letters, digits, '_' or '-': {value!r}")
    return value


def _new_run_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dt%H%M%S%f")
    return f"{timestamp}z-{os.getpid()}-{uuid4().hex[:8]}"
