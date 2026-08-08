# `extra/ground_station/presentation/terminal.py`

## 1. Introduction

Terminal implementation used by the canonical operator-side application
command, including keyboard-driven manual drive with bounded periodic command
transmission.

## 2. Code structure

| Definition                                                                                              | Inputs                                                         | Output                                                | Algorithm or purpose                                                                              |
| ------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `main(argv: list[str] \| None = None) -> int`                                                           | Local config path plus optional CLI overrides                  | Process status code                                   | Loads operator listener/UI settings, starts a local server, and runs interactive or plain-terminal workflow. |
| `GroundStationTerminal(server, dashboard, command_handler, *, ...)`                                     | Server, dashboard, command parser, and UI/timing configuration | Terminal instance                                     | Owns terminal-local parsing, refresh, activity logging, and manual-key state.                     |
| `run()` / `_start_background_tasks()` / `_refresh_loop()`                                               | Terminal application and local timing                          | UI exit code or scheduled tasks                       | Run the prompt-toolkit UI and periodic dashboard refresh.                                         |
| `_manual_input_loop()` / `_submit(buffer)` / `_execute_command(text)`                                   | Key/typed input                                                | Command handling and activity entries                 | Parse text through `GroundStationCommandHandler` and report delivery results.                     |
| `_start_manual_drive(vehicle_id)` / `_stop_manual_drive(send_stop)`                                     | Target ID and stop flag                                        | Manual-mode state and optional stop command           | Enable/disable manual drive and safely end an active session.                                     |
| `_manual_active()` / `_set_manual_axis(...)` / `_send_manual_input_if_due()` / `_manual_key_state(now)` | Local key state, axes, and monotonic time                      | Active flag, bounded axes, or `MANUAL_INPUT` delivery | Maintain recent local axes and transmit at configured cadence.                                    |
| `_record(message)` / `_drain_activity()` / `_request_exit(app)`                                         | Activity message or application                                | Updated activity stream or graceful exit              | Manage terminal activity and UI shutdown.                                                         |
| `_operator_input_loop(...)` / `_run_manual_drive(...)`                                                  | Plain-terminal server/parser and manual input                  | Command loop result                                   | Provide non-UI input and manual-drive fallback.                                                   |
| `_wait_for_ack(...)` / `_last_reported_state(...)`                                                      | Server, vehicle ID, and optional timeout                       | Acknowledgement or latest state text                  | Wait for asynchronous result or read dashboard-visible state.                                     |
| `_redraw(text)` / `_windows_key_down(virtual_key)`                                                      | Terminal text or virtual-key code                              | Terminal redraw or key-down Boolean                   | Implement console redraw and Windows key polling.                                                 |

## 3. Special data and cross-references

Text commands become [[commands|VehicleCommand]] via
[[command-handler|GroundStationCommandHandler]]. Manual throttle/steering are
terminal-local latest input sent at a bounded cadence; their freshness is later
enforced by the vehicle controller/runtime.

## 4. Position in the project

Calls [[configuration|GroundStationConfiguration]] and
[[server|GroundStationServer]] only. It never accesses a vehicle runtime,
fleet manager, controller, or IO adapter.

## 5. Use and verification

`test/unit_test_ground_station.py` covers command syntax and dashboard-visible
state. `test/test_integration_ground_station.py` covers the connected command
path; manual-drive behavior is exercised through the same typed command route.
