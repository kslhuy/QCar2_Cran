# Generic MCU/SoC and HIL targets

The electronics runtime is vendor-neutral. `STM32F405 + STM32H753` is one
manifest profile for the current PCB, not a requirement of the algorithm or HIL
architecture.

## Runtime boundaries

```text
Vehicle simulator
  ├─ PCB sensor and power models
  ├─ sensor_compute link model
  └─ BoardFirmwareCore contract
       ├─ python_reference                  SIL reference
       ├─ native_cpp DLL/SO                 native SIL
       └─ hil_external
            └─ HILTransport
                 ├─ UDP (included)
                 ├─ UART/CAN gateway
                 ├─ PCIe/shared memory
                 └─ project-specific link
                         │
                 any real MCU or SoC
                         │
                 portable C++ core
                 + vendor BSP/HAL adapter
```

The C++ core never calls STM32 HAL, an RTOS, Linux or a network API. The target
application supplies `cran_platform_ops` from
[`cran_target_adapter.h`](native/cran_target_adapter.h):

- `monotonic_time_ns(context)`;
- `write_channel(context, vehicle_or_v2v, payload, length)`;
- an optional logger.

Sensor/input events are pushed through `cran_target_on_sensor` and
`cran_target_on_input`. `cran_target_poll` drains algorithm outputs through the
platform callback. This contract works with bare metal, FreeRTOS, Zephyr,
ThreadX, QNX or Linux.

The allocation-free [`cran_hil_protocol.h`](native/cran_hil_protocol.h)
implements the exact same `CRHL` header and CRC32 as Python, so a target does
not need a JSON or Python runtime for data-plane framing. JSON is used only for
the low-rate capability/status payloads.

## Hardware manifests

Profiles included with the simulator are:

- `cran_segula_stm32`;
- `generic_dual_mcu`;
- `generic_single_mcu`;
- `linux_soc`;
- `desktop_sil`.

A profile supplies defaults only. A fully described custom target is accepted:

```yaml
electronics:
  hardware_target:
    profile: nxp_s32k_project
    target_id: qcar_nxp_01
    topology: dual_node
    sensor_node:
      node_id: acquisition
      platform: nxp_s32k144
      architecture: arm_cortex_m4f
      runtime: zephyr
      execution: hil_external
      float_width_bits: 32
      transports: [spi, can_fd]
      features: [sensor_acquisition, timestamping]
    compute_node:
      node_id: estimator
      platform: nxp_s32k344
      architecture: arm_cortex_m7
      runtime: zephyr
      execution: hil_external
      float_width_bits: 64
      max_payload_bytes: 4096
      transports: [can_fd, ethernet]
      features: [firmware_core, trust, observer, v2v]
```

For one Raspberry Pi, Jetson, Zynq Linux processing system or other consolidated
SoC, use `topology: single_node` and give both roles the same `node_id`. The
digital twin then uses one power/reset state machine and does not double-count
the node's current.

## External HIL configuration

The included UDP transport is non-blocking and uses the versioned `CRHL` binary
protocol with sequence number, simulation timestamp, payload length and CRC32:

```yaml
electronics:
  hardware_target:
    profile: linux_soc
    target_id: qcar_compute_01
  sensor_compute_interface: ethernet
  vehicle_interface: can_fd
  firmware:
    backend: hil_external
    hil:
      transport: udp
      remote_host: 192.168.1.50
      remote_port: 9100
      local_host: 0.0.0.0
      local_port: 9101
      strict_handshake: true
      required_features: [firmware_core, trust, observer, v2v]
      min_float_width_bits: 64
      min_payload_bytes: 1500
```

The simulator and hardware exchange manifests before outputs are accepted. The
handshake checks protocol schema, required features, numeric precision, payload
capacity, memory and dynamic-allocation support. It deliberately does not
require a vendor, CPU architecture or operating system. The current C++ core
uses the C++17 standard library and dynamic containers; very small MCUs that do
not meet those declared resources are rejected explicitly. A later fixed-memory
core profile can relax that capability without changing the HIL boundary.

For UART, USB, CAN or PCIe, implement the five-method Python `HILTransport`
protocol (`send`, `receive`, `reset`, `close`, `get_status`) or bridge that link
to UDP. The `CRHL` framing remains unchanged, so no estimator code changes.

## Cross-compiling the core

Desktop builds produce both the DLL/SO and a static library. A target toolchain
can build only the static artifact:

```powershell
cmake -S electronics/native -B electronics/native/build-target `
  -DCRAN_BUILD_SHARED=OFF `
  -DCRAN_BUILD_STATIC=ON `
  -DCMAKE_TOOLCHAIN_FILE=path/to/target-toolchain.cmake
cmake --build electronics/native/build-target
```

Link `cran_electronics_core_static` with the project BSP and implement
`cran_platform_ops`. The supplied C ABI avoids C++ name mangling at the HAL
boundary. A constrained target may declare 32-bit floating point, but a
64-bit requirement in the manifest handshake will then reject it explicitly
instead of silently changing estimator behavior.

## Acceptance sequence for a new target

1. Describe the node(s) and capabilities in YAML.
2. Cross-compile and link the static core with its BSP/HAL.
3. Implement or bridge one `HILTransport`.
4. Confirm `HIL handshake: READY` in Ground Station.
5. Replay the same sensor, V2V, Trust and Observer validation vectors.
6. Accumulate the configured zero-failure parity gate.
7. Enable Native authority manually; keep automatic Python failback active.

A real MCU/SoC connected through this boundary is processor/HIL. Running the
same executable as another process on the simulator PC remains SIL or
process-in-the-loop, even though it uses the identical protocol.
