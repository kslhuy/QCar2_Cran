# CRAN/SEGULA Electronics Digital Twin

This package models the PCB as a separate system from `MockQCar`. `MockQCar`
still owns the vehicle plant and its existing vehicle sensors. The electronics
twin receives only physical plant truth and generates its own measurements,
timing, noise, bias, faults and communication traffic.

## Implemented vertical slice

```text
vehicle dynamics (plant truth only)
                |
                v
PCB sensor models: IIM-42652 + NEO-M9N + IIS2MDC
                |
          sensor_node role
                |
        SPI or UART bus model
                |
          compute_node role  <---- firmware/Trust/Observer/V2V
                |
       UART or CAN/CAN-FD model
                |
 VehicleObserver auxiliary_sensors["electronics"]
                |
    optional, explicitly configured fusion
```

The active V2V data path can also traverse the electronics twin:

```text
V2VManager MsgPack
  -> vehicle UART/CAN model
  -> compute-target firmware core
  -> V2V radio model (rate/delay/jitter/loss/BER)
  -> UDP peer
  -> remote compute-target firmware core
  -> remote vehicle UART/CAN model
  -> unchanged V2VManager/Trust message handlers
```

The two roles may be separate MCUs or one consolidated MCU/SoC. The legacy
`nav_mcu`/`com_mcu` names remain aliases for existing application code. The twin
currently includes:

- 12 V input, 5 V buck and 3.3 V LDO functional power model, load and brownout;
- hardware-manifest-driven sensor/compute node power, reset and boot state;
- independent IMU, GNSS and magnetometer clocks, quantization, white noise,
  bias random walk, dropout, freeze, bias and increased-noise faults;
- deterministic transaction-level UART, SPI and CAN/CAN-FD with serialization,
  arbitration, delay, jitter, packet loss, bit errors and CAN CRC rejection;
- the current 18-byte NAV IMU packet (`legacy_v1`) and a versioned full-sensor
  packet with CRC32 (`sensor_frame_v2`);
- a replaceable firmware-core boundary plus a Python reference COM application;
- status data suitable for Ground Station telemetry.
- an authoritative bidirectional COM-firmware V2V path, while preserving the
  existing MsgPack schemas, clean/attacked channels and reference timestamps.
- a platform-neutral C++ Trust/Distributed Observer kernel, including the
  stateful EMA/MATLAB Dirichlet memory, attack flags, local evidence geometric
  aggregation and circular-state consensus correction;
- a fail-safe binding that first replays Python Trust/Observer stages in shadow,
  then permits explicit C++ authority only after the configured four-stage
  parity gate; every authoritative result remains cross-checked and one mismatch
  immediately fails back to Python.

This is a transaction/functional-level twin. It does not attempt analog SPICE,
RF propagation, pin-level MCU instruction emulation or PCB thermal simulation.
Those are separate fidelity layers and can be connected later without changing
the vehicle integration contract.

## Configuration

The default fake-vehicle parameters enable the twin in auxiliary mode:

```yaml
electronics:
  enabled: true
  nav_protocol: sensor_frame_v2
  nav_com_interface: spi
  vehicle_interface: uart
  fusion:
    mode: auxiliary
  v2v:
    enabled: true
    mode: firmware
```

`auxiliary` never overwrites vehicle sensor fields. `prefer_electronics` replaces
supported IMU/GNSS fields when a valid PCB frame is available.
`weighted_fusion` uses `electronics_weight` to blend them. This distinction is
deliberate: PCB sensors are auxiliary measurements, not the existing vehicle
sensor mock.

V2V has two migration modes:

- `firmware`: raw MsgPack goes through the vehicle link, COM core and radio
  model in both directions. This is the default for fake vehicles.
- `mirror`: the legacy host remains authoritative, while COM receives a copy.
  Use it as the A/B baseline when migrating Trust and estimation code.

Bus faults can be changed at runtime:

```python
twin.set_bus_faults("nav_com", fixed_delay_s=0.004, jitter_s=0.001)
twin.set_bus_faults("vehicle", drop_probability=0.05, bit_error_rate=1e-6)
twin.set_sensor_fault("gnss", "dropout")
twin.set_input_voltage(5.0)
```

Radio faults are controlled by the attached `ElectronicsV2VBridge` or directly
from the Ground Station `Compute ↔ V2V Radio` selection. Bit corruption is rejected
by the simulated link CRC, so upper layers observe packet loss instead of
invalid MsgPack objects.

## Firmware path: Python to C/C++ and hardware-in-the-loop

`BoardFirmwareCore` is the stable boundary. Both a Python implementation and a
working `ctypes` C++ shared-library backend are included. The native ABI stays
small and deterministic:

```c
void *cran_core_create(uint32_t vehicle_id, double publish_rate_hz);
void  cran_core_reset(void *core);
void  cran_core_on_nav(void *core, /* normalized sensor fields */);
void  cran_core_on_input(void *core, uint8_t channel, const uint8_t *data,
                         size_t len, uint64_t t_ns);
void  cran_core_step(void *core, uint64_t t_ns);
int   cran_core_pop_output(void *core, uint8_t channel, uint8_t *dst, size_t capacity);
void  cran_core_destroy(void *core);

int cran_observer_weights(/* trust sources, target flags, config, outputs */);
int cran_observer_correct(/* direct/neighbor states and weights, output */);
int cran_observer_predict(/* model, control, clean/anchor context, output */);
```

Keep HAL calls outside this core. Any MCU/SoC build supplies a BSP/HAL adapter;
the digital twin supplies virtual UART/SPI/CAN/Ethernet/shared-memory and time. The same Trust,
Distributed Observer and later controller code can then run in:

1. native software-in-the-loop on the PC;
2. any MCU, RTOS target or Linux SoC connected to the simulator (HIL);
3. the final vehicle with real sensors and V2V radio.

The present board profile exposes sensor-compute SPI/UART and external UART/USB.
Selecting CAN in the twin is useful for architecture testing, but physical CAN
requires a CAN/CAN-FD transceiver and connector/protection in a PCB revision.

## Migration milestones

1. **Implemented:** independent PCB sensors, power/MCU state and faultable
   UART/SPI/CAN transaction models.
2. **Implemented:** route existing V2V datagrams through the vehicle bus, COM
   firmware and a faultable radio model without changing the MsgPack contract.
3. **Implemented:** extract the Trust temporal memory and Distributed Observer
   correction into a no-Python/no-network C++ core, bind it in shadow mode and
   verify deterministic Python/C++ parity, including replay and angle wrapping.
4. **Implemented:** move startup/equal/paper/trust-based adaptive weight
   construction, neighbor caps, flag adaptation, direct recovery and all active
   prediction variants behind the same ABI. Ground Station now exposes separate
   Trust/weight/correction/prediction parity and an evidence-count authority gate.
5. **Implemented for SIL:** safety-gated Native authority, automatic Python
   failback, Ground Station mode/reset controls, repeatable fault presets and a
   JSON-producing end-to-end manual validation suite.
6. **Implemented:** generic hardware manifests, single/dual-node topology,
   capability handshake, CRC-framed HIL protocol, UDP/adapter transport,
   portable C platform operations and shared/static CMake targets.
7. **Next hardware-fidelity layer:** move the remaining sensor-specific Trust
   evidence and Mahalanobis checks into C++, then deploy the same static core on
   the selected compute target.
7. Validate with SIL, processor/HIL and finally two physical boards before
   moving any controller command path onto the PCB.

The staged boundary is deliberate. Sensor-specific innovation/Mahalanobis
checks still produce `local_trust_sample` and `global_trust_sample` in Python.
The native core owns their temporal Trust/Dirichlet update, constructs adaptive
weights, performs circular consensus correction and propagates the corrected
state through the configured prediction model. A missing DLL or native runtime
error reports `available: false` and cannot stop or alter the Python observer.
The authority gate remains blocked until every stage has the configured minimum
number of zero-failure comparisons. Passing the gate never switches silently:
the operator must request `native_authority`, and continuous parity monitoring
automatically returns authority to Python on the first mismatch or native error.

Build the included native reference on Windows, then select it in YAML:

```powershell
cmake -S electronics/native -B electronics/native/build -G Ninja
cmake --build electronics/native/build
```

```yaml
electronics:
  firmware:
    backend: native_cpp
    library_path: electronics/native/build/cran_electronics_core.dll
    publish_rate_hz: 20.0
```

Run the deterministic tests from the `qcar` directory:

```powershell
python -m unittest electronics.test_electronics_digital_twin -v
python -m unittest electronics.test_v2v_transport_integration -v
python -m unittest electronics.test_trust_observer_native -v
python -m unittest electronics.test_hardware_abstraction -v
```

For the complete repeatable suite and web-app test flow, see
[`MANUAL_TEST.md`](MANUAL_TEST.md), or run from the repository root:

```powershell
.\run_electronics_manual_test.ps1
```

For custom MCU/SoC manifests, cross-compilation and external HIL transport, see
[`HARDWARE_TARGETS.md`](HARDWARE_TARGETS.md).
