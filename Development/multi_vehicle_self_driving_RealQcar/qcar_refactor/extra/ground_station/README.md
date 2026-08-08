# Ground-station side program

`extra.ground_station` is the operator-side TCP listener and presentation
program. It routes typed `VehicleCommand` requests to registered vehicle
runtimes and displays received monitoring/LiDAR diagnostic data. It does not
deploy files, create vehicle hardware, or call a vehicle runtime directly.

## Fixed YAML location

The ground station always loads
`extra/ground_station/config/ground_station.yaml`; there is no template,
local copy, or `--config` selector. The committed default listens on
`0.0.0.0:5000`, accepts up to 4096-byte frames, refreshes the dashboard at
4 Hz, and refreshes the LiDAR viewer at 20 Hz. This is separate from
`config/config_ground_station.yaml`, which is the vehicle-runtime bridge
profile. A selected vehicle bridge must target the ground-station machine and
port `5000` with a compatible frame limit.

## Canonical commands

Run the normal terminal, listener, dashboard, typed-command surface, and
integrated Qt LiDAR viewer:

```powershell
python -m extra.ground_station terminal
```

Run only the TCP listener when another operator frontend will use the same
server process:

```powershell
python -m extra.ground_station server
```

`--host`, `--port`, display-rate, and duration flags are temporary process
overrides for diagnostics and tests; no option is persisted. These are the
only supported operator entry points.

## Ownership

- `app.py` and `__main__.py`: the only operator entry points; select
  `terminal` or `server`.
- `configuration.py`: load and validate the fixed application YAML path.
- `ground_station_type.py`: shared configuration, request, session, and
  command-delivery data contracts.
- `core/`: listener lifecycle, TCP session registry, and typed command routing.
- `presentation/`: terminal workflow and read-only dashboard rendering.
- `utils/`: shared logging and optional live diagnostic plotting. The plotting
  utilities receive already-normalized values; they never access vehicle IO.

```text
extra/ground_station/
  app.py, configuration.py       entry point and fixed-YAML loader
  ground_station_type.py         side-program data contracts
  core/                          listener, session registry, command routing
  presentation/                 terminal and dashboard
  utils/                        logging and live plotting support
  config/ground_station.yaml    the only application configuration file
```

The vehicle-side TCP bridge remains in `utils/ground_station/`; runtime module
selection remains in `config/config_ground_station.yaml` and vehicle profiles.
