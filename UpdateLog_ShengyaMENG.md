# Update Log 2025/12/16 (Shengya Meng)
- Added `initPlatoon.py` to create platoons with variable sizes.
- Enabled observer configuration via external setting files (see details below).
- Added `DistributedLuenbergerEstimator` in `fleet_state_estimators.py`.

## ToDo List
- [ ] Test if the observer gain can be used correctly in the observer class;


## Observer config flow (summary)

- `config_main.py` now defines `ObserverConfig`, so YAML/JSON `observer` blocks load into `config.observer` with defaults (types and gains always present).
- `VehicleObserverSimple._get_observer_config` merges external observer settings with defaults and accepts scalar/vector/matrix values for `observer_gain` and `consensus_gain`.
- `VehicleLogic` should read `config.observer` when constructing `VehicleObserver`, so per-vehicle `local_estimator_type` and `fleet_estimator_type` from `configs/carX.yaml` take effect instead of hardcoded defaults.
- `_create_fleet_estimator` forwards gains from `self.observer_config` to `FleetEstimatorFactory`; fleet estimators in `fleet_state_estimators.py` consume these values on construction.

## Example run

```bash
python vehicle_main.py --car-id 0 --config configs/car0.yaml
```
