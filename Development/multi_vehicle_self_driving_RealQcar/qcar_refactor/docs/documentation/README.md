# Project Documentation

This folder is the code-oriented companion to `PROJECT_ARCHITECTURE.md`.
It documents the active Python runtime, its executable integration layer under
`extra/`, and current launch configuration under `config/`. Legacy material
under `refs/` and test implementation remain out of scope.

## Reading order

1. [System mechanism](system-mechanism.md) explains ownership and the control-loop flow.
2. [Documentation standard](documentation-standard.md) defines page scope and symbols.
3. [Documentation TODO](TODO.md) tracks the agreed principles, completed work,
   and remaining physical-vehicle/documentation work.
4. [Source-page template](source-page-template.md) and
   [package README template](package-readme-template.md) define the common
   layout.
5. [Source-tree map](source-tree.md) maps the active code tree to this reference.
6. [Active API inventory](api-inventory.md) lists classes and functions across
   the active core and utility modules.
7. [Core](core/) documents the domain contracts and lifecycle.
8. [Utilities](utils/) documents adapters and algorithms.
9. [Extra integration](extra/) documents launchers, scenario parsing, CARLA,
   and the ground-station application.
10. [Configuration](config/) documents current profiles and operator scenarios.

## Source-to-page rule

Every substantive active `core/*.py`, `utils/**/*.py`, and `extra/**/*.py`
module has one matching page. Package `__init__.py` files are import shims and
are indexed rather than repeated. Configuration has one index page: current
profiles and operator scenarios are described, while `config/scenarios/test/`
is intentionally excluded until it is documented alongside its tests.

The folder layout mirrors the source tree:

```text
engineering_reference/
  core/                         <- core/*.py
  utils/control/{controller,managers,observer,path_planner}/
  utils/fleet/{distributed_observer}/
  utils/{ground_station,io,v2v}/
  extra/{deployment,ground-station,platform}/ <- extra/**/*.py
  config/                       <- current profiles and operator scenarios
```

## Configuration classification

Current/operator scenarios live in `config/scenarios/`. Test-only scenarios
live in `config/scenarios/test/`; their routes, ports, and timing are not
operator defaults and are documented with their test code rather than here.
