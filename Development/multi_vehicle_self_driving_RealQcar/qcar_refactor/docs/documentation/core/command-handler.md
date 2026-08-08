# `core/command_handler.py`

## 1. Introduction

`VehicleCommandHandler` converts a validated [[commands|VehicleCommand]]
into a `CommandHandling` plan. It owns command eligibility and policy; the
[[vehicle-runtime|VehicleRuntime]] applies profile changes and performs every
requested safe zero write.

## 2. Code structure

`CommandHandling` stores the acknowledgement plus optional reset/safe-stop,
controller profile/manual input, and planner SDCS-route actions. The handler
is initialized with local vehicle ID, `StateMachine`, planner manager, optional
fleet manager, and whether a manual controller profile exists.

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `CommandHandling` | result and optional action fields | frozen action plan | Separates eligibility result from runtime-owned side effects. |
| `VehicleCommandHandler.__init__(...)` | vehicle ID, state machine, planner, fleet, manual capability | handler | Retains policy collaborators and initializes manual-disabled mode. |
| `reset()` | none | none | Disables manual mode when runtime restarts. |
| `handle(command, now_monotonic)` | typed command and optional monotonic time | `CommandHandling` | Checks target, dispatches manual/fleet/SDCS/path/lifecycle rules, and requests safe actions. |
| `_handle_sdcs_map_command(command)` | SDCS enable/disable command | plan or `None` | Rejects active followers, selects `sdcs_map` in READY/RUNNING, and restores configured planner safely on disable. |
| `_handle_manual_command(command)` | manual enable/disable/input command | plan or `None` | Requires configured manual profile, allowed state, non-follower fleet role, and RUNNING input. |
| `_disable_manual()` | none | configured profile name or `None` | Clears manual mode and requests restoration only when it was enabled. |
| `_fleet_blocks_manual()` | fleet phase/role | boolean | Blocks manual control only for active/building followers. |
| `_fleet_is_active()` | fleet phase | boolean | Identifies building or active fleet operation. |
| `_fleet_blocks_sdcs_map()` | fleet phase/role | boolean | Blocks SDCS route changes only for active/building followers. |
| `_handle_fleet_command(command, now_monotonic)` | typed command and monotonic time | plan or `None` | Delegates fleet lifecycle; converts rejection/cancel into command result and requested safe stop. |
| `_result(command, outcome, reason_code, reason)` | command/outcome/reasons | `CommandHandling` | Wraps a generated acknowledgement in a no-extra-action plan. |
| `_command_result(command, outcome, reason_code, reason)` | command/outcome/reasons | `CommandResult` | Creates an acknowledgement with local vehicle ID and current state. |

## 3. Special data and cross-references

`CommandHandling.sdcs_node_sequence` is a tuple of validated map IDs; `sdcs_loop`
is `0`, `1`, `2`, or `"inf"`. `require_safe_stop` and `safe_stop_reason` are
requests to runtime, never writes made by this class. A fleet follower retains
its fleet-owned controller reference, so it cannot select the SDCS route or
manual controller while fleet operation is active.

## 4. Position in the project

[[vehicle-runtime|VehicleRuntime.handle_command]] is the direct caller.
The handler consults [[vehicle-state-machine|StateMachine]], planner manager,
and fleet manager, but it never writes [[vehicle-types|ControlInput]], starts
modules, or changes a profile itself.

## 5. Use and verification

Call `handle()` only with a `VehicleCommand` already validated by
[[commands|commands.py]]; then let runtime apply every returned action.
`test/unit_test_command_handler.py` covers lifecycle, manual, fleet, SDCS
leader/follower eligibility, route-loop transfer, and rejection reasons.
