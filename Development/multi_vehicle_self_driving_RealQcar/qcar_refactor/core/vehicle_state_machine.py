"""
Minimal State Machine for QCar Vehicle Control

States: INITIALIZING, READY, RUNNING, STOPPED, ERROR

Transitions:
  INITIALIZING -> CALIBRATING (calibration in progress)
  INITIALIZING -> READY    (startup complete)
  INITIALIZING -> ERROR    (startup failure)
  CALIBRATING -> READY    (calibration complete)
  CALIBRATING -> ERROR    (calibration failure)
  READY -> RUNNING         (external command sends START)
  READY -> STOPPED         (external command sends STOP or EMERGENCY_STOP)
  READY -> ERROR           (observer/planner/IO failure)
  RUNNING -> STOPPED       (external command sends STOP, path finished, or emergency)
  RUNNING -> ERROR         (observer/planner/controller/IO failure)
  STOPPED -> READY         (external command sends RESET)
  ERROR -> READY           (external command sends RESET, modules healthy)

Safety: Only RUNNING allows non-zero throttle and steering.
"""

from enum import Enum, auto
from core.commands import CommandType, VehicleCommand


class State(Enum):
    INITIALIZING = auto()
    CALIBRATING = auto()
    READY = auto()
    RUNNING = auto()
    STOPPED = auto()
    ERROR = auto()


class StateMachine:
    def __init__(self):
        self._state = State.INITIALIZING
        self._error_reason: str = ""

    # ---- Properties ----

    @property
    def state(self) -> State:
        return self._state

    # ---- Safety checks ----
    def calibrating(self) -> bool:
        """True when the system is in the calibration phase."""
        return self._state == State.CALIBRATING

    def should_drive(self) -> bool:
        """Only RUNNING is allowed to produce non-zero throttle and steering."""
        return self._state == State.RUNNING

    def safe_command_required(self) -> bool:
        """True when the runtime must force zero throttle/steering."""
        return self._state != State.RUNNING

    # ---- State transitions ----

    def mark_ready(self) -> None:
        """Call after all modules started successfully."""
        if self._state == State.INITIALIZING:
            self._state = State.READY

    def mark_error(self, reason: str) -> None:
        """Call when any critical module fails."""
        self._error_reason = reason
        self._state = State.ERROR

    def handle_command(self, cmd: VehicleCommand) -> bool:
        """Process one validated command and report whether it was applied."""
        c = cmd.command_type

        if c == CommandType.START:
            if self._state == State.READY:
                self._state = State.RUNNING
                return True

        elif c == CommandType.STOP:
            if self._state in (State.READY, State.RUNNING):
                self._state = State.STOPPED
                return True

        elif c == CommandType.EMERGENCY_STOP:
            # Emergency stop from any state (except ERROR which is already dead)
            if self._state != State.ERROR:
                self._state = State.STOPPED
                self._error_reason = str(cmd.payload.get("reason", "Emergency stop command"))
                return True

        elif c == CommandType.RESET:
            if self._state in (State.STOPPED, State.ERROR):
                self._state = State.READY
                self._error_reason = ""
                return True
        return False

    def get_status(self) -> dict:
        return {
            "state": self._state.name,
            "error_reason": self._error_reason,
        }
