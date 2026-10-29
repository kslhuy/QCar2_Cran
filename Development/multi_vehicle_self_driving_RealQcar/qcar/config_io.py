"""Safe YAML mappings shared by runtime observer and controller configuration.

Cache parsed files by their filesystem revision. Return an independent copy so
per-vehicle overrides cannot change the defaults seen by another vehicle.
"""
from copy import deepcopy
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml

_SAFE_LOADER = getattr(yaml, "CSafeLoader", yaml.SafeLoader)


@lru_cache(maxsize=128)
def _read_revision(path: str, modified_ns: int, size: int) -> dict[str, Any]:
    with open(path, encoding="utf-8-sig") as handle:
        value = yaml.load(handle, Loader=_SAFE_LOADER)
    if not isinstance(value, dict):
        raise ValueError(f"Configuration must contain a YAML mapping: {path}")
    return value


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    """Read a mapping, invalidating the cache when the file changes."""
    resolved = Path(path).resolve(strict=True)
    revision = resolved.stat()
    return deepcopy(_read_revision(str(resolved), revision.st_mtime_ns, revision.st_size))
