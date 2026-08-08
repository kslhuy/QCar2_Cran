# `utils/control/managers/manager_base.py`

## 1. Introduction

`ManagerBase` owns the configured utility and explicitly allowed lazy runtime profiles.

## 2. Code structure

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `ManagerBase(configured_utility, builders=None)` | Required utility and optional named lazy builders | Manager state or `ValueError` | Validates configured utility, reserves `configured`, and stores only allowed builders. |
| `active_name` | Manager state | Active profile `str` | Returns configured or runtime-selected profile name. |
| `has_profile(name)` / `is_selected(name)` | Profile name | `bool` | Test availability or active selection. |
| `select(name)` | Non-empty allowed profile name | Managed utility or `KeyError`/`ValueError` | Lazily builds, validates, caches, selects, and returns a profile. |
| `restore_configured()` | No inputs | Configured utility | Re-selects the static configured profile. |
| `_active` | Manager state | Active managed utility | Internal selected utility accessor. |
| `_validate_utility(utility, name)` | Candidate utility and diagnostic name | Validation side effect | Abstract subclass contract check. |

## 3. Special data and cross-references

Profile identity is separate from profile parameters; lazy builders are only those factory exposes.

## 4. Position in the project

Managers are selected by core command/runtime policy and do not create state transitions.

## 5. Use and verification

Manager behavior is covered by controller/observer/planner/runtime tests.
