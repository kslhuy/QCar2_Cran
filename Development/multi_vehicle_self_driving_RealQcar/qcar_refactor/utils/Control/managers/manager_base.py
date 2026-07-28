"""Shared lazy-selection support for runtime control-utility managers."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Mapping
from typing import Generic, TypeVar


ManagedUtility = TypeVar("ManagedUtility")


class ManagerBase(ABC, Generic[ManagedUtility]):
    """Own configured and optional lazily-built utility implementations."""

    CONFIGURED = "configured"

    def __init__(
        self,
        configured_utility: ManagedUtility,
        builders: Mapping[str, Callable[[], ManagedUtility]] | None = None,
    ) -> None:
        self._validate_utility(configured_utility, "configured_utility")
        self._utilities: dict[str, ManagedUtility] = {self.CONFIGURED: configured_utility}
        self._builders = dict(builders or {})
        if self.CONFIGURED in self._builders:
            raise ValueError("'configured' is reserved for the configured utility")
        self._active_name = self.CONFIGURED

    @property
    def active_name(self) -> str:
        """Return the active configured or runtime-selected profile name."""
        return self._active_name

    def has_profile(self, name: str) -> bool:
        """Return whether a named profile is available for selection."""
        return name in self._utilities or name in self._builders

    def is_selected(self, name: str) -> bool:
        """Return whether a profile is currently active."""
        return self._active_name == name

    def select(self, name: str) -> ManagedUtility:
        """Select and, when needed, lazily construct an allowed profile."""
        if not isinstance(name, str) or not name:
            raise ValueError("utility profile name must be a non-empty string")
        utility = self._utilities.get(name)
        if utility is None:
            builder = self._builders.get(name)
            if builder is None:
                raise KeyError(f"Utility profile is not configured: '{name}'")
            utility = builder()
            self._validate_utility(utility, f"Utility builder '{name}' result")
            self._utilities[name] = utility
        self._active_name = name
        return utility

    def restore_configured(self) -> ManagedUtility:
        """Restore the profile selected by the static vehicle configuration."""
        return self.select(self.CONFIGURED)

    @property
    def _active(self) -> ManagedUtility:
        return self._utilities[self._active_name]

    @abstractmethod
    def _validate_utility(self, utility: object, name: str) -> None:
        """Reject values that do not implement this manager's public contract."""
