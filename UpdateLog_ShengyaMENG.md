# Update Log 2026/01/08 (Shengya MENG)
- [ ] Check the reason why the fleet state is **nan** in the log. And fix it. 
- [x] Fixed the one reason causing nan. Correct the discrete update. But it **still diffuse**. 
```
x_i_new = x_vec + dt * (dynamics_term + measurement_term - consensus_term)
```
- [x]Fixed the warning. It is beacuse we stop the car at the different time. 
```
2026-01-08 16:50:53 - [Car Car_1] - WARNING - _distributed_luenberger_observer_update:1107 - Vehicle 1: Using fallback strategy for neighbor 2
```


## What I did and what I got

### Setting the observer state always be 0
```
x_i_new = np.zeros_like(dynamics_term)  # Testing without update first.
```
- The data log file seems correct. Not diffuse. Means **The communication is no problem**
- the log of each vehicle can hadle the fleet state message correctly, like 
```
2026-01-08 11:23:28 - [Car Car_3] - INFO - _handle_fleet_state_message:440 -     vehicle_0: x=0.000, y=0.000, theta=0.000, v=0.000, conf=0.80
```
- But in the log file of vehicle 1 and vehicle 2, there is no fleet data got from the neighbor, like 
```
2026-01-08 11:23:33 - [Car Car_1] - WARNING - _distributed_luenberger_observer_update:1102 - Vehicle 1: No fleet_state from neighbor 2, building from scratch
```

**The Nan maybe from the "get the fleet data from the communication"**

**Or the algrithem, if the gain is applied corretly?**






# Update Log 2026/01/07 (Shengya MENG)
- [x] Transfer the fleet state to be distributed observer state. [_transfer_estimated_states_to_fleet_states function_](../qcar/Observer/fleet_state_estimators.py)
- [x]Config the communication network, in my own distributed observer. [get_neighbors](../qcar/Observer/fleet_state_estimators.py). In the log of each vehicle, it is configed correctly. 
- [ ] Check the reason why the fleet state is **nan** in the log. And fix it. 


# Update Log 2026/01/06 (Shengya Meng)
- [x] Transfer the estimated state from the distributed observer to be fleet state. [_transfer_estimated_states_to_fleet_states function_](../qcar/Observer/fleet_state_estimators.py)
    - The estimated state from the distributed observer and fleet state have different meanings.
    - Implemented the conversion method to map the distributed observer's estimated state to the fleet state format.
    - Clarified that pi-1 and di0 should be sourced from local data for accurate calculations.
- [x] Change the config file [carx.yaml](../configs/car0.yaml) to be first config file higher priority than [fleet_config.yaml](../configs/fleet_config.yaml). Modified `VehicleObserverSimple` to log observer configuration upon initialization for better debugging and tracking.
- [x] Remove the manual setting  of the initial location of each vehicle in `initPlatoon.py`. Instead, set the initial distance between vehicles and calculate their positions accordingly.

## ToDo List
- [ ] Config the **leader without distributed observer**. But keep it state in the fleet state. So that the followers can use it to calculate the relative position and velocity to the leader.
- [ ] Check the reason why the fleet state is **nan** in the log. And fix it. 
- [ ] Use fake vehicle to test.
- [ ] Get the latest control input from V2V messages
- [ ] get the latest relative position from sensors (Lidar/Camera)

# Update Log 2025/12/17 (Shengya Meng)
- [x] Test if the observer gain can be used correctly in the observer class
    - local observer type can be applied. 

## ToDo List
- [ ] Make a logs for observer , local and fleet 
- [ ] Distinguiwish the observer gain for local observer and distributed osberver

# Update Log 2025/12/16 (Shengya Meng)
- Added `initPlatoon.py` to create platoons with variable sizes.
- Enabled observer configuration via external setting files (see details below).
- Added `DistributedLuenbergerEstimator` in `fleet_state_estimators.py`.

## ToDo List
- [ ] Test if the observer gain can be used correctly in the observer class;
- [ ] Make a logs for observer , local and fleet 

## HUY UPDATE 

Refactor state estimators and V2V manager for improved configuration handling and logging
- ( GOOD ) Use fleet state received to Fleet Estimator (Check Consensus) 
- Updated EKFStateEstimator and LuenbergerStateEstimator to accept a configuration dictionary for parameters. "config_fleet_estimators.yaml" "config_local_estimators.yaml"

So i think no need to do " python vehicle_main.py --car-id 0 --config configs/car0.yaml"
Just change the type in "config_fleet_estimators.yaml" , so all the vehicle have the same fleet estimator type



- Removed unused GPS parameter from EKFStateEstimator.
- Introduced ConsensusFleetEstimator for distributed fleet state estimation.
- Enhanced V2VManager by normalizing state data and improving message handling.
- Commented out legacy queue handling in V2VManager to streamline data processing.

## Observer config flow (summary)

- `config_main.py` now defines `ObserverConfig`, so YAML/JSON `observer` blocks load into `config.observer` with defaults (types and gains always present).
- `VehicleObserverSimple._get_observer_config` merges external observer settings with defaults and accepts scalar/vector/matrix values for `observer_gain` and `consensus_gain`.
- `VehicleLogic` should read `config.observer` when constructing `VehicleObserver`, so per-vehicle `local_estimator_type` and `fleet_estimator_type` from `configs/carX.yaml` take effect instead of hardcoded defaults.
- `_create_fleet_estimator` forwards gains from `self.observer_config` to `FleetEstimatorFactory`; fleet estimators in `fleet_state_estimators.py` consume these values on construction.

## Example run

```bash
python vehicle_main.py --car-id 0 --config configs/car0.yaml
```
