# Linux-vehicle deployment CLI

This package is deliberately separate from vehicle and ground-station configuration.
It packages only explicitly selected files, verifies their manifest locally and on the
target, stages a versioned release, and activates it atomically.  It does not start a
vehicle program during upload.

`DeploymentTarget` is a generic Linux-host endpoint: SSH/SFTP access, release
directory, health requirements, and opaque start/stop commands. It does not
encode QCar hardware or a runtime fleet formation; the selected bundle and
start command choose the platform. Copy `config/templates/target.example.yaml` to a
local path outside this repository, or start from the current
`config/templates/qcar_target.example.yaml`. Configure a least-privilege SSH key and
a pinned `known_hosts` file; do not commit passwords, private keys, hosts, or
deployment destinations.

## Configuration boundary

`extra/deployment` is an operator-side program, not a vehicle module. Its
configuration is therefore kept with the program instead of in `config/`,
which selects vehicle-runtime modules:

- `config/bundles/` contains reviewed, versioned allowlists for release input.
- `config/templates/` contains safe examples committed to Git.
- `config/local/` contains ignored endpoint inventories, host keys, output
  archives, and credential *references* for the current operator machine.

Each `deployment_target` has four explicit groups: logical `vehicle_id`,
`ssh` connection/authentication settings, `release` location/process commands,
and `preflight` requirements. A `deployment_targets` inventory chooses one
`bundle` (`specification` and `output`) and an ordered `targets` list. This
keeps deployment transport distinct from vehicle configuration and from the
runtime `utils/fleet` formation.

## Guided deployment-test flow

The safe deployment-test bundle includes only `core`, `utils`, the selected
config profiles, and no origin route data; its vehicle profile sets
`mission.path: null`.

### 1. Build the local archive (does not contact the QCar)

```powershell
python -m extra.deployment package --bundle-spec extra/deployment/config/bundles/qcar_deployment_test.bundle.yaml --output $env:TEMP\qcar-release.tar.gz
```

This creates `$env:TEMP\qcar-release.tar.gz`. The reported file count, size,
and manifest hash describe the exact files that would be sent later.

### 2. Verify the local archive (does not contact the QCar)

```powershell
python -m extra.deployment verify --bundle $env:TEMP\qcar-release.tar.gz
```

The hash and file count must match step 1. At this point nothing has left the
ground-station computer.

`config/bundles/qcar_runtime.bundle.yaml` is also route-free. Add an approved
project-owned route asset only to a dedicated operational bundle; do not
package files from `refs/` for normal physical deployment.

### 3. Run remote preflight (read-only SSH checks)

Configure the SSH key named in your local target file, set `password_env`, or
set `password_prompt: true`. The prepared local QCar target uses
`password_prompt: true`, so the CLI asks for a hidden password directly. Then run:

```powershell
python -m extra.deployment preflight --target-file extra/deployment/config/local/qcar-61231.target.local.yaml
```

Preflight checks the pinned host identity, hostname, free space, and required
commands. It does not create, upload, or start anything. If an SSH session drops
during these read-only checks, the CLI reconnects once; uploads are never retried
automatically.

### 4. Check the upload request without uploading

```powershell
python -m extra.deployment deploy --target-file extra/deployment/config/local/qcar-61231.target.local.yaml --vehicle-id <confirmed-id> --host <confirmed-host> --bundle $env:TEMP\qcar-release.tar.gz --yes --dry-run
```

`--dry-run` does not connect to the QCar. It only checks that the typed host and
vehicle ID match the selected local target file.

### 5. Upload and activate the release (mutates the QCar)

Every remote mutation requires the target configuration and repeated operator target
selection. After confirming the logical vehicle ID and host, remove `--dry-run`:

```powershell
python -m extra.deployment deploy --target-file extra/deployment/config/local/qcar-61231.target.local.yaml --vehicle-id 0 --host 192.168.1.230 --bundle $env:TEMP\qcar-release.tar.gz --yes
```

This uploads to a staged release, verifies the manifest on the QCar, and atomically
updates `current`. It does not start the vehicle. `start` and `stop` are separate
commands with the same target safeguards. Use `logs` to tail the configured log and
`fetch-artifact` to retrieve a path relative to the active release. `rollback --release
release-<manifest-prefix>` re-verifies and atomically reactivates a prior release.

### 6. Start the safe physical-QCar runtime (separate, explicit operation)

The deployment target now starts `extra.platform.qcar.process_runner`, which creates the Quanser
`QCar(readMode=1)` object and gives it to the existing `IOQCar2` adapter.  It
does **not** inject a runtime START command. `IOQCar2.stop()` sends the normal
neutral command during runtime shutdown, and PAL `QCar.terminate()` sends the
final neutral command during cleanup, including an SSH `stop` (`SIGTERM`).

After deploying a bundle produced by the current test bundle policy, first
review the request without connecting:

```powershell
python -m extra.deployment start --target-file extra/deployment/config/local/qcar-61231.target.local.yaml --vehicle-id 0 --host 192.168.1.230 --yes --dry-run
```

Then, only while the QCar is safely supported with its wheels clear, start the
neutral validation process:

```powershell
python -m extra.deployment start --target-file extra/deployment/config/local/qcar-61231.target.local.yaml --vehicle-id 0 --host 192.168.1.230 --yes
python -m extra.deployment logs --target-file extra/deployment/config/local/qcar-61231.target.local.yaml
```

The supplied `qcar-61231.target.local.yaml` points at
`config_vehicle_deployment_test.yaml`. That profile has `mission.path: null`
and `target_velocity: 0.0`, and its `tcp_qcar_lan` bridge connects to the
configured ground station for registration and monitoring. It has no route,
but the bridge can receive operator commands, so treat it as a no-motion
IOQCar2/PAL and ground-station validation profile. Stop it explicitly:

```powershell
python -m extra.deployment stop --target-file extra/deployment/config/local/qcar-61231.target.local.yaml --vehicle-id 0 --host 192.168.1.230 --yes
```

An operational route needs a separately reviewed vehicle profile, a valid
ground-station command source, and a deliberate START command.  Do not change
the safe local target to an operational profile until those controls are ready.

## One-command multi-target upload

Copy `config/templates/deployment_targets.example.yaml` to the ignored
`extra/deployment/config/local/deployment_targets.local.yaml`. The provided
local file already lists `qcar-61231.target.local.yaml`. For each additional
Linux endpoint, add one separately pinned local target file and list it under
`targets`; its `vehicle_id` and host remain defined only in that file.

Review the entire deployment-target plan without connecting:

```powershell
python -m extra.deployment targets-deploy --targets-file extra/deployment/config/local/deployment_targets.local.yaml --yes --dry-run
```

Run read-only checks for every vehicle, then package once and upload that exact
verified archive sequentially to every configured vehicle:

```powershell
python -m extra.deployment targets-preflight --targets-file extra/deployment/config/local/deployment_targets.local.yaml
python -m extra.deployment targets-deploy --targets-file extra/deployment/config/local/deployment_targets.local.yaml --yes
```

The archive is stored at the `bundle.output` path in the deployment-targets file (currently
`extra/deployment/config/local/qcar-targets-release.tar.gz`). `targets-deploy` never
starts a vehicle. Once each result has been checked, `targets-start` is the
separate explicit action:

```powershell
python -m extra.deployment targets-start --targets-file extra/deployment/config/local/deployment_targets.local.yaml --yes --dry-run
python -m extra.deployment targets-start --targets-file extra/deployment/config/local/deployment_targets.local.yaml --yes
python -m extra.deployment targets-stop --targets-file extra/deployment/config/local/deployment_targets.local.yaml --yes
```

`targets-start` and `targets-stop` return one structured result per vehicle with
readiness, health, collected logs, shutdown outcome, and any failure reason.

## Read-only physical-QCar IO capture

The physical QCar can be sampled without starting its vehicle runtime. Stop a
running vehicle first, physically support it with its wheels clear, then use
SSH to run the capture from the active release. The capture only calls the
QCar sensor-read path; it does not issue an actuator command. PAL still sends
its normal final neutral cleanup when the device session is released.

```powershell
ssh -i "$env:USERPROFILE\.ssh\qcar_deploy_ed25519" nvidia@192.168.1.230 'cd /home/nvidia/Documents/qcar_refactor/current/payload && python3 -m extra.platform.qcar.io_capture capture --duration-s 10 --sample-rate-hz 50'
```

Fetch the CSV and metadata, then plot on the ground station:

```powershell
python -m extra.deployment fetch-artifact --target-file extra/deployment/config/local/qcar-61231.target.local.yaml --artifact artifacts/real_qcar_io --output-dir test
python -m extra.platform.qcar.io_capture plot --input test/artifacts/real_qcar_io/real_qcar_io.csv
```

The resulting CSV, JSON metadata, and PNG are evidence for the physical IO
integration test. The `io_capture` module is already included through the
`extra/platform/qcar` bundle entry; deploy the current bundle before capture.

### Verify the real `IOQCar2` LiDAR path

After the direct raw LiDAR probe below has confirmed the installed sensor
orientation and ranges, run this separate stationary integration capture. It
opens the QCar and the selected direct PAL LiDAR client, lets `IOQCar2` start
its local polling worker, and records the worker's normalized
`LaserScanSample` frames alongside the IMU/tachometer data. It starts no
vehicle route and calls no `QCar.write`; PAL performs only its normal session
cleanup when the resources are released. Run `targets-stop` first and keep
the vehicle physically supported/stationary.

```powershell
ssh -i "$env:USERPROFILE\.ssh\qcar_deploy_ed25519" nvidia@192.168.1.230 'cd /home/nvidia/Documents/qcar_refactor/current/payload && python3 -m extra.platform.qcar.io_capture capture --duration-s 10 --sample-rate-hz 50 --with-lidar'
python -m extra.deployment fetch-artifact --target-file extra/deployment/config/local/qcar-61231.target.local.yaml --artifact artifacts/real_qcar_io --output-dir test
python -m extra.platform.qcar.io_capture plot --input test/artifacts/real_qcar_io/real_qcar_io.csv
python -m extra.platform.qcar.io_capture plot-lidar --input test/artifacts/real_qcar_io/real_qcar_lidar_normalized.csv
```

The result directory contains `real_qcar_io.csv` for IMU/wheel data,
`real_qcar_lidar_normalized.csv` for the new IO-produced scan bins, the
matching PNGs, and `real_qcar_io.json` with the local worker counters. This
is the physical proof that the QCar IO adapter has a local LiDAR producer; it
does not yet enable localisation or send raw scans to the ground station.

### Live Qt LiDAR viewer through the ground station

`PyQt5` and `pyqtgraph` provide a small visual component inside the existing
terminal ground station. It uses the registered vehicle TCP connection; it
does not use a deployment target, SSH, a callback address, or UDP.

#### Ground-station CLI switch

Start the normal ground-station CLI after the vehicle runtime has registered.
No deployment target, callback IP, or UDP firewall rule is needed for this
normal display path.



```powershell
python -m extra.ground_station terminal
```

At the `ground-station>` prompt, use:

```text
lidar status
lidar start 0
lidar stop 0
```

`lidar start` sends `ENABLE_LIDAR_DIAGNOSTIC` through the existing TCP vehicle
bridge. For QCar, that command creates PAL LiDAR on the vehicle and starts its
local IO worker; it is deliberately **not** created by `targets-start`. CARLA
enables its existing local producer. The runtime forwards only the newest
normalized scan at the configured diagnostic rate (20 Hz by default), while
the Qt viewer has its own 20 Hz refresh loop. `lidar stop`, closing the viewer,
or closing the terminal sends `DISABLE_LIDAR_DIAGNOSTIC`; QCar then stops its
worker and releases PAL LiDAR. This is not a driving command and sends no
actuator input. It is valid in `READY`, `RUNNING`, and `STOPPED`, so a
stationary vehicle can be inspected without issuing `start 0`.

#### Low-level fallback

For isolated network troubleshooting, start the standalone viewer on the
ground-station PC first:

```powershell
python -m extra.ground_station.utils.live_plotting --host 0.0.0.0 --port 5001
```

If Windows has no existing inbound rule for UDP 5001, add one once in an
Administrator PowerShell window:

```powershell
New-NetFirewallRule -DisplayName "QCar sensor diagnostics UDP 5001" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 5001 -RemoteAddress 192.168.1.230 -Profile Any
```

Then run a bounded ten-second capture from the QCar, replacing the host with
the ground-station LAN address (`192.168.1.135` in the current setup):

```powershell
ssh -i "$env:USERPROFILE\.ssh\qcar_deploy_ed25519" nvidia@192.168.1.230 'cd /home/nvidia/Documents/qcar_refactor/current/payload && python3 -m extra.platform.qcar.io_capture capture --duration-s 10 --sample-rate-hz 50 --with-lidar --diagnostic-host 192.168.1.135 --diagnostic-port 5001'
```

The viewer renders the normalized 2D scan in real time. Its camera panel is a
clearly labelled placeholder for a future `CameraFrame` producer; no camera
stream has been added yet.

For password authentication, the tool prompts once for each target.  SSH keys
are the practical choice when deploying to several QCars.

## Read-only physical-QCar LiDAR capture

The LiDAR probe is separate from the QCar IO capture. It opens only PAL's
direct `QCarLidar` client, calls `read()`, copies the `angles` and `distances`
arrays, and closes that LiDAR client. It never creates a `QCar` actuator or a
`QCarGPS` object, so it sends no driving command and does not alter the
LiDAR-to-GPS service or calibration. Stop any vehicle runtime or other LiDAR
client first; the QCar must remain stationary for this initial acquisition
test.

Deploy the current bundle, then run the ten-second test on the QCar:

```powershell
ssh -i "$env:USERPROFILE\.ssh\qcar_deploy_ed25519" nvidia@192.168.1.230 'cd /home/nvidia/Documents/qcar_refactor/current/payload && python3 -m extra.platform.qcar.lidar_capture capture --duration-s 10 --poll-rate-hz 20'
```

Fetch the resulting raw scan table and metadata, then create a Cartesian PNG
on the ground station:

```powershell
python -m extra.deployment fetch-artifact --target-file extra/deployment/config/local/qcar-61231.target.local.yaml --artifact artifacts/real_qcar_lidar --output-dir test
python -m extra.platform.qcar.lidar_capture plot --input test/artifacts/real_qcar_lidar/real_qcar_lidar.csv
```

The artifact contains every raw point with capture time, scan index, angle in
radians, distance in metres, and validity. It validates PAL geometry before
the separate `IOQCar2 --with-lidar` integration capture above; neither command
is a localisation correction.

### Enable LiDAR in the normal QCar runtime

After both captures look correct, set the confirmed `angle_offset_rad` /
`angle_sign` in `config/config_io.yaml`, deploy, and start normally:

```powershell
python -m extra.deployment targets-deploy --targets-file extra/deployment/config/local/deployment_targets.local.yaml --yes
python -m extra.deployment targets-start --targets-file extra/deployment/config/local/deployment_targets.local.yaml --yes
```

This starts only the QCar runtime; it does not create PAL LiDAR, install a
launcher, or start a second program on the vehicle. Open the ground-station
viewer and issue `lidar start 0` when an explicit diagnostic scan is wanted.
The worker's scan queue remains local; the bounded display copy is not yet
connected to scan-matching/localisation, so enabling the viewer cannot change
current driving behaviour.

Directory layout:

- `bundle.py`, `remote.py`, and `configuration.py`: deployment services.
- `deployment_type.py`: the side-program's shared immutable bundle, target,
  command-result, preflight, and multi-target data contracts.
- `cli.py` and `__main__.py`: command-line entry points.
- `config/bundles/`: versioned file-selection policies.
- `config/templates/`: safe-to-commit configuration examples.
- `config/local/`: ignored machine-specific targets, host keys, and target inventories.
