# Ground-Station Application

## 1. Introduction

This extra package is the operator-facing server, terminal, dashboard, and
local application configuration. It is separate from vehicle-side transport
utilities and from the vehicle bridge profile.

## 2. File structure and variations

[core server](core/server.md) owns TCP sessions; [command handler](core/command-handler.md)
parses CLI text; [dashboard](presentation/dashboard.md) renders monitoring;
[terminal](presentation/terminal.md) provides interaction; [application entry](app.md)
selects terminal/server; [configuration](configuration.md) validates local
fixed-path operator settings; and [core listener](core/listener.md) owns
headless process lifecycle. [Live plotting](utils/live-plotting.md) and
[logging](utils/logging.md) remain optional operator-side support.
[Ground-station types](ground-station-type.md) own the shared dataclasses.

## 3. Shared data and cross-references

The application sends [[commands|VehicleCommand]] and displays acknowledgements
and monitoring data using the common ground-station protocol.

## 4. Position in the project

It is an external operator process. [[runtime-facade|GroundStationRuntimeFacade]]
on the vehicle is the corresponding boundary; neither side bypasses core safety.

## 5. Use and verification

Edit the one reviewed YAML at `extra/ground_station/config/ground_station.yaml`,
then run `python -m extra.ground_station terminal` or
`python -m extra.ground_station server`. Verify with
`test/unit_test_ground_station.py` and integration tests.

## Conclusion

All operator files share framed command/monitoring data; they differ by session,
parsing, or presentation role while vehicle runtime retains authority.
