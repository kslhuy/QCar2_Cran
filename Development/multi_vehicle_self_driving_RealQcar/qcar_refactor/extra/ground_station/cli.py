"""Terminal ground station for live monitoring and typed vehicle commands."""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
import ctypes
import os
import queue
import sys
import threading
import time

from core.commands import CommandError
from core.commands import CommandType, VehicleCommand

from .command_handler import GroundStationCommandHandler
from .dashboard import GroundStationDashboard
from .server import GroundStationServer


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the QCar refactor ground-station terminal")
    parser.add_argument("--host", default="0.0.0.0", help="TCP listener address")
    parser.add_argument("--port", type=int, default=5000, help="TCP listener port")
    parser.add_argument("--refresh-hz", type=float, default=4.0, help="dashboard refresh rate")
    parser.add_argument("--stale-after-s", type=float, default=1.0, help="mark snapshots stale after this age")
    parser.add_argument("--no-dashboard", action="store_true", help="log server only")
    parser.add_argument("--no-input", action="store_true", help="disable interactive command input")
    parser.add_argument("--duration-s", type=float, default=None, help="stop automatically after this duration")
    parser.add_argument("--ack-timeout-s", type=float, default=3.0, help="operator wait time for COMMAND_ACK")
    args = parser.parse_args(argv)
    if args.refresh_hz <= 0.0:
        parser.error("--refresh-hz must be positive")
    if args.duration_s is not None and args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    if args.ack_timeout_s <= 0.0:
        parser.error("--ack-timeout-s must be positive")

    server = GroundStationServer(args.host, args.port)
    dashboard = GroundStationDashboard(stale_after_s=args.stale_after_s)
    command_handler = GroundStationCommandHandler()
    terminal_ui_enabled = not args.no_dashboard and sys.stdout.isatty()
    if terminal_ui_enabled:
        terminal = GroundStationTerminal(
            server,
            dashboard,
            command_handler,
            listen_host=args.host,
            refresh_hz=args.refresh_hz,
            duration_s=args.duration_s,
            acknowledgement_timeout_s=args.ack_timeout_s,
            input_enabled=not args.no_input,
        )
        try:
            server.start()
            return terminal.run()
        finally:
            server.stop()

    if not args.no_dashboard:
        print("Ground-station dashboard disabled because stdout is not an interactive terminal")

    # The non-interactive/log-only mode deliberately avoids terminal UI dependencies.
    stop_requested = threading.Event()
    input_thread: threading.Thread | None = None
    if not args.no_input:
        input_thread = threading.Thread(
            target=_operator_input_loop,
            args=(server, command_handler, stop_requested, args.ack_timeout_s),
            name="GroundStationOperatorInput",
            daemon=True,
        )
    try:
        server.start()
        print(f"Ground station listening on {args.host}:{server.port}")
        if input_thread is not None:
            input_thread.start()
        started_at = time.monotonic()
        refresh_period_s = 1.0 / args.refresh_hz
        while not stop_requested.is_set():
            if args.duration_s is not None and time.monotonic() - started_at >= args.duration_s:
                break
            if terminal_ui_enabled:
                _redraw(dashboard.render(server.session_rows()))
            time.sleep(refresh_period_s)
    except KeyboardInterrupt:
        return 0
    finally:
        stop_requested.set()
        server.stop()
    return 0


class GroundStationTerminal:
    """One prompt-toolkit event loop with independent status, activity, and input panes."""

    def __init__(
        self,
        server: GroundStationServer,
        dashboard: GroundStationDashboard,
        command_handler: GroundStationCommandHandler,
        *,
        listen_host: str,
        refresh_hz: float,
        duration_s: float | None,
        acknowledgement_timeout_s: float,
        input_enabled: bool,
    ) -> None:
        from prompt_toolkit.application import Application
        from prompt_toolkit.filters import Condition
        from prompt_toolkit.key_binding import KeyBindings
        from prompt_toolkit.layout import HSplit, Layout
        from prompt_toolkit.layout.containers import ConditionalContainer
        from prompt_toolkit.styles import Style
        from prompt_toolkit.widgets import Frame, TextArea

        self._server = server
        self._listen_host = listen_host
        self._dashboard = dashboard
        self._command_handler = command_handler
        self._refresh_period_s = 1.0 / refresh_hz
        self._duration_s = duration_s
        self._acknowledgement_timeout_s = acknowledgement_timeout_s
        self._input_enabled = input_enabled
        self._stop_requested = False
        self._activity = deque(maxlen=200)
        self._pending_activity: queue.SimpleQueue[str] = queue.SimpleQueue()
        self._last_reported_context: dict[int, str] = {}
        self._manual_lock = threading.Lock()
        self._manual_vehicle_id: int | None = None
        self._manual_throttle = 0.0
        self._manual_steering = 0.0
        self._manual_throttle_until = 0.0
        self._manual_steering_until = 0.0
        self._next_manual_send = 0.0

        self._status_pane = TextArea(
            text="Waiting for vehicle registrations…",
            read_only=True,
            scrollbar=True,
            focusable=False,
            style="class:status",
        )
        self._activity_pane = TextArea(
            text="",
            read_only=True,
            scrollbar=True,
            focusable=False,
            style="class:activity",
        )
        self._input_pane = TextArea(
            prompt="ground-station> ",
            multiline=False,
            height=1,
            style="class:input",
            accept_handler=self._submit,
        )

        bindings = KeyBindings()

        @bindings.add("c-c")
        def _exit(event) -> None:
            self._request_exit(event.app)

        manual_active = Condition(self._manual_active)

        @bindings.add("up", filter=manual_active)
        def _manual_forward(event) -> None:
            self._set_manual_axis(throttle=0.35)

        @bindings.add("down", filter=manual_active)
        def _manual_reverse(event) -> None:
            self._set_manual_axis(throttle=-0.35)

        @bindings.add("left", filter=manual_active)
        def _manual_left(event) -> None:
            self._set_manual_axis(steering=0.30)

        @bindings.add("right", filter=manual_active)
        def _manual_right(event) -> None:
            self._set_manual_axis(steering=-0.30)

        @bindings.add(" ", filter=manual_active)
        def _manual_zero(event) -> None:
            self._set_manual_axis(throttle=0.0, steering=0.0, hold=False)

        @bindings.add("q", filter=manual_active)
        def _manual_exit(event) -> None:
            self._stop_manual_drive(send_stop=True)

        root = HSplit(
            [
                Frame(self._status_pane, title="Vehicle status"),
                Frame(self._activity_pane, title="Commands and acknowledgements"),
                ConditionalContainer(
                    Frame(self._input_pane, title="Command input"),
                    filter=Condition(lambda: self._input_enabled),
                ),
            ]
        )
        self._app = Application(
            layout=Layout(root, focused_element=self._input_pane if input_enabled else self._status_pane),
            key_bindings=bindings,
            full_screen=True,
            mouse_support=False,
            style=Style.from_dict({"frame.border": "#888888", "input": "bg:#202020"}),
        )

    def run(self) -> int:
        self._record(f"Ground station listening on {self._listen_host}:{self._server.port}")
        try:
            self._app.run(pre_run=self._start_background_tasks)
        except KeyboardInterrupt:
            pass
        return 0

    def _start_background_tasks(self) -> None:
        self._app.create_background_task(self._refresh_loop())
        self._app.create_background_task(self._manual_input_loop())

    async def _refresh_loop(self) -> None:
        started_at = time.monotonic()
        while not self._stop_requested:
            if self._duration_s is not None and time.monotonic() - started_at >= self._duration_s:
                self._request_exit(self._app)
                return
            rows = self._server.session_rows()
            self._status_pane.text = self._dashboard.render(rows)
            for row in rows:
                vehicle_id = row["vehicle_id"]
                context = _last_reported_state(self._server, vehicle_id)
                if self._last_reported_context.get(vehicle_id) != context:
                    self._last_reported_context[vehicle_id] = context
                    self._record(f"Vehicle {vehicle_id} report changed: {context}")
            self._drain_activity()
            self._app.invalidate()
            await asyncio.sleep(self._refresh_period_s)

    async def _manual_input_loop(self) -> None:
        """Transmit physical manual-key state at 20 Hz independently of display refresh."""
        while not self._stop_requested:
            self._send_manual_input_if_due()
            await asyncio.sleep(0.05)

    def _submit(self, buffer) -> bool:
        text = buffer.text.strip()
        buffer.reset()
        if text:
            threading.Thread(
                target=self._execute_command,
                args=(text,),
                name="GroundStationCommand",
                daemon=True,
            ).start()
        return True

    def _execute_command(self, text: str) -> None:
        self._record(f"> {text}")
        try:
            request = self._command_handler.parse(text)
        except (ValueError, CommandError) as exc:
            self._record(f"Command rejected: {exc}")
            return
        if request.action in {"quit", "exit"}:
            self._request_exit(self._app)
            return
        if request.action == "help":
            self._record("start|stop|emergency-stop|reset|build-fleet|cancel-fleet <vehicle-id>")
            self._record("set-velocity <id> <m/s> | set-path <id> <csv-path> | manual <id> <throttle> <steering>")
            self._record("enable-sdcs-map <id> <0|1|2|inf> <node> <node> [...] | disable-sdcs-map <id>")
            self._record("enable-manual <id> | disable-manual <id> | manual-drive <id> | list | status <id> | quit")
            return
        if request.action == "list":
            self._record(f"Registered vehicles: {[row['vehicle_id'] for row in self._server.session_rows()]}")
            return
        if request.action == "status":
            assert request.vehicle_id is not None
            row = next((item for item in self._server.session_rows() if item["vehicle_id"] == request.vehicle_id), None)
            self._record(str(row) if row is not None else f"Vehicle {request.vehicle_id} is not registered")
            return
        if request.action == "manual-drive":
            assert request.vehicle_id is not None
            self._start_manual_drive(request.vehicle_id)
            return
        assert request.vehicle_id is not None and request.command is not None
        delivery = self._command_handler.route(self._server, request)
        if not delivery.accepted:
            self._record(f"Command not sent: {delivery.reason}")
            return
        if request.command.command_type == CommandType.MANUAL_INPUT:
            self._record(f"Manual input sent to vehicle {request.vehicle_id}")
            return
        self._record(
            f"Command {request.command.command_id} sent to vehicle {request.vehicle_id} "
            f"{_last_reported_state(self._server, request.vehicle_id)}; awaiting COMMAND_ACK"
        )
        result = _wait_for_ack(
            self._server,
            request.vehicle_id,
            request.command.command_id,
            self._acknowledgement_timeout_s,
        )
        if result is None:
            self._record(f"Command {request.command.command_id} has no acknowledgement after {self._acknowledgement_timeout_s:.1f} s")
            return
        reason = f" ({result.get('reason_code')}: {result.get('reason')})" if result.get("reason") else ""
        self._record(
            f"COMMAND_ACK {request.command.command_id}: {result.get('outcome')}{reason} "
            f"{_last_reported_state(self._server, request.vehicle_id)}"
        )

    def _start_manual_drive(self, vehicle_id: int) -> None:
        if self._manual_active():
            self._record("Manual drive is already active; press Q to stop it")
            return
        command = VehicleCommand(CommandType.ENABLE_MANUAL)
        delivery = self._server.send_command(vehicle_id, command)
        if not delivery.accepted:
            self._record(f"Manual control not enabled: {delivery.reason}")
            return
        result = _wait_for_ack(self._server, vehicle_id, command.command_id, self._acknowledgement_timeout_s)
        if result is None or result.get("outcome") != "applied":
            reason = "no acknowledgement" if result is None else str(result.get("reason", "manual enable rejected"))
            self._record(f"Manual control not enabled: {reason}")
            return
        with self._manual_lock:
            self._manual_vehicle_id = vehicle_id
            self._next_manual_send = 0.0
        self._record("Manual drive active: arrows control throttle/steering, Space zeros input, Q stops the vehicle")

    def _manual_active(self) -> bool:
        with self._manual_lock:
            return self._manual_vehicle_id is not None

    def _set_manual_axis(
        self,
        *,
        throttle: float | None = None,
        steering: float | None = None,
        hold: bool = True,
    ) -> None:
        now = time.monotonic()
        with self._manual_lock:
            if throttle is not None:
                self._manual_throttle = throttle
                self._manual_throttle_until = now + 0.15 if hold else 0.0
            if steering is not None:
                self._manual_steering = steering
                self._manual_steering_until = now + 0.15 if hold else 0.0

    def _send_manual_input_if_due(self) -> None:
        now = time.monotonic()
        with self._manual_lock:
            vehicle_id = self._manual_vehicle_id
            if vehicle_id is None or now < self._next_manual_send:
                return
            throttle, steering = self._manual_key_state(now)
            self._next_manual_send = now + 0.05
        delivery = self._server.send_command(
            vehicle_id,
            VehicleCommand(CommandType.MANUAL_INPUT, {"throttle": throttle, "steering": steering}),
        )
        if not delivery.accepted:
            self._record(f"Manual input stopped: {delivery.reason}")
            self._stop_manual_drive(send_stop=False)

    def _manual_key_state(self, now: float) -> tuple[float, float]:
        """Return simultaneous physical arrow-key state on Windows.

        Terminal key events do not expose independent key-up state, so they
        cannot reliably represent combinations such as Up+Left. The Windows
        API does; non-Windows terminals retain the short key-event fallback.
        """
        if os.name == "nt":
            if _windows_key_down(0x20):  # VK_SPACE
                return 0.0, 0.0
            throttle = 0.35 if _windows_key_down(0x26) else -0.35 if _windows_key_down(0x28) else 0.0
            steering = 0.30 if _windows_key_down(0x25) else -0.30 if _windows_key_down(0x27) else 0.0
            return throttle, steering
        throttle = self._manual_throttle if now <= self._manual_throttle_until else 0.0
        steering = self._manual_steering if now <= self._manual_steering_until else 0.0
        return throttle, steering

    def _stop_manual_drive(self, *, send_stop: bool) -> None:
        with self._manual_lock:
            vehicle_id = self._manual_vehicle_id
            self._manual_vehicle_id = None
            self._manual_throttle = 0.0
            self._manual_steering = 0.0
        if vehicle_id is not None and send_stop:
            self._server.send_command(vehicle_id, VehicleCommand(CommandType.STOP, {"reason": "manual_drive_exit"}))
        self._record("Manual drive stopped")

    def _record(self, message: str) -> None:
        self._pending_activity.put(message)
        self._app.invalidate()

    def _drain_activity(self) -> None:
        while True:
            try:
                self._activity.append(self._pending_activity.get_nowait())
            except queue.Empty:
                break
        self._activity_pane.text = "\n".join(self._activity)
        self._activity_pane.buffer.cursor_position = len(self._activity_pane.text)

    def _request_exit(self, app) -> None:
        self._stop_requested = True
        self._stop_manual_drive(send_stop=True)
        app.exit(result=0)


def _operator_input_loop(
    server: GroundStationServer,
    command_handler: GroundStationCommandHandler,
    stop_requested: threading.Event,
    acknowledgement_timeout_s: float,
) -> None:
    while not stop_requested.is_set():
        try:
            text = input("ground-station> ").strip()
        except (EOFError, KeyboardInterrupt):
            stop_requested.set()
            return
        try:
            request = command_handler.parse(text)
        except (ValueError, CommandError) as exc:
            print(f"Command rejected: {exc}")
            continue
        if request.action in {"quit", "exit"}:
            stop_requested.set()
            return
        if request.action == "help":
            print("Use: start|stop|emergency-stop|reset|build-fleet|cancel-fleet <vehicle-id>")
            print("     set-velocity <vehicle-id> <m/s> | set-path <vehicle-id> <csv-path>")
            print("     enable-sdcs-map <vehicle-id> <0|1|2|inf> <node> <node> [...] | disable-sdcs-map <vehicle-id>")
            print("     enable-manual <vehicle-id> | disable-manual <vehicle-id>")
            print("     manual <vehicle-id> <throttle> <steering-rad> | manual-drive <vehicle-id>")
            continue
        if request.action == "list":
            print(f"Registered vehicles: {[row['vehicle_id'] for row in server.session_rows()]}")
            continue
        if request.action == "status":
            assert request.vehicle_id is not None
            row = next((item for item in server.session_rows() if item["vehicle_id"] == request.vehicle_id), None)
            print(row if row is not None else f"Vehicle {request.vehicle_id} is not registered")
            continue
        if request.action == "manual-drive":
            assert request.vehicle_id is not None
            _run_manual_drive(server, request.vehicle_id, stop_requested, acknowledgement_timeout_s)
            continue
        assert request.vehicle_id is not None and request.command is not None
        delivery = command_handler.route(server, request)
        command = request.command
        vehicle_id = request.vehicle_id
        if delivery.accepted:
            if command.command_type == CommandType.MANUAL_INPUT:
                print(f"Manual input sent to vehicle {vehicle_id}")
                continue
            print(
                f"Command {command.command_id} sent to vehicle {vehicle_id} "
                f"{_last_reported_state(server, vehicle_id)}; awaiting COMMAND_ACK"
            )
            result = _wait_for_ack(server, vehicle_id, command.command_id, acknowledgement_timeout_s)
            if result is None:
                print(f"Command {command.command_id} has no acknowledgement after {acknowledgement_timeout_s:.1f} s")
            else:
                reason = f" ({result.get('reason_code')}: {result.get('reason')})" if result.get("reason") else ""
                print(
                    f"COMMAND_ACK {command.command_id}: {result.get('outcome')}{reason} "
                    f"{_last_reported_state(server, vehicle_id)}"
                )
        else:
            print(f"Command not sent: {delivery.reason}")


def _run_manual_drive(
    server: GroundStationServer,
    vehicle_id: int,
    stop_requested: threading.Event,
    acknowledgement_timeout_s: float,
) -> None:
    """Send bounded keyboard input as typed commands at a fixed 20 Hz rate."""
    try:
        import msvcrt
    except ImportError:
        print("manual-drive currently requires a Windows console; use 'manual <id> <throttle> <steering>' instead")
        return
    enable_command = VehicleCommand(CommandType.ENABLE_MANUAL)
    enable = server.send_command(vehicle_id, enable_command)
    if not enable.accepted:
        print(f"Manual control not enabled: {enable.reason}")
        return
    result = _wait_for_ack(server, vehicle_id, enable_command.command_id, acknowledgement_timeout_s)
    if result is None or result.get("outcome") != "applied":
        reason = "no acknowledgement" if result is None else str(result.get("reason", "manual enable rejected"))
        print(f"Manual control not enabled: {reason}")
        return
    throttle = 0.0
    steering = 0.0
    throttle_until = 0.0
    steering_until = 0.0
    hold_window_s = 0.15
    print("Manual drive: Arrow Up/Down throttle, Arrow Left/Right steering, Space zero, Q stop and exit")
    next_send = time.monotonic()
    while not stop_requested.is_set():
        while msvcrt.kbhit():
            key = msvcrt.getwch()
            if key == "q":
                server.send_command(
                    vehicle_id,
                    VehicleCommand(CommandType.STOP, {"reason": "manual_drive_exit"}),
                )
                return
            now = time.monotonic()
            if key in ("\\x00", "\\xe0"):
                key = msvcrt.getwch()
                if key == "H":
                    throttle = 0.35
                    throttle_until = now + hold_window_s
                elif key == "P":
                    throttle = -0.35
                    throttle_until = now + hold_window_s
                elif key == "K":
                    steering = 0.30
                    steering_until = now + hold_window_s
                elif key == "M":
                    steering = -0.30
                    steering_until = now + hold_window_s
            elif key == " ":
                throttle = 0.0
                steering = 0.0
                throttle_until = 0.0
                steering_until = 0.0
        now = time.monotonic()
        if now >= next_send:
            active_throttle = throttle if now <= throttle_until else 0.0
            active_steering = steering if now <= steering_until else 0.0
            delivery = server.send_command(
                vehicle_id,
                VehicleCommand(
                    CommandType.MANUAL_INPUT,
                    {"throttle": active_throttle, "steering": active_steering},
                ),
            )
            if not delivery.accepted:
                print(f"Manual input stopped: {delivery.reason}")
                return
            next_send = now + 0.05
        time.sleep(0.005)


def _wait_for_ack(
    server: GroundStationServer,
    vehicle_id: int,
    command_id: str,
    timeout_s: float,
) -> dict[str, object] | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for row in server.session_rows():
            if row["vehicle_id"] != vehicle_id:
                continue
            result = row["last_command_result"]
            if isinstance(result, dict) and result.get("command_id") == command_id:
                return result
        time.sleep(0.02)
    return None


def _last_reported_state(server: GroundStationServer, vehicle_id: int) -> str:
    """Format the target's latest monitoring snapshot, never a synchronous read.

    The snapshot may lag a command acknowledgement; naming that fact prevents
    the terminal log from implying it queried the vehicle's live internals.
    """
    row = next((item for item in server.session_rows() if item["vehicle_id"] == vehicle_id), None)
    snapshot = row.get("snapshot") if isinstance(row, dict) else None
    if not isinstance(snapshot, dict):
        return "(last reported snapshot: runtime=?, fleet=unavailable)"
    runtime = str(snapshot.get("runtime_state", "?"))
    fleet = snapshot.get("fleet_summary")
    if not isinstance(fleet, dict) or not fleet.get("configured", False):
        fleet_state = "unavailable"
    else:
        phase = str(snapshot.get("fleet_phase", "?"))
        role = str(fleet.get("role", "?"))
        order = fleet.get("member_order")
        members = fleet.get("member_count")
        fleet_state = f"{phase}/{role}"
        if isinstance(order, int) and isinstance(members, int) and members > 0:
            fleet_state += f" order={order + 1}/{members}"
        reason = fleet.get("reason")
        if isinstance(reason, str) and reason:
            fleet_state += f" reason={reason}"
    return f"(last reported snapshot: runtime={runtime}, fleet={fleet_state})"


def _redraw(text: str) -> None:
    # Windows Terminal and standard ANSI terminals redraw without creating a log flood.
    sys.stdout.write("\x1b[2J\x1b[H" + text + "\n")
    sys.stdout.flush()


def _windows_key_down(virtual_key: int) -> bool:
    """Return whether a Windows virtual key is physically held down."""
    return bool(ctypes.windll.user32.GetAsyncKeyState(virtual_key) & 0x8000)


if __name__ == "__main__":
    raise SystemExit(main())
