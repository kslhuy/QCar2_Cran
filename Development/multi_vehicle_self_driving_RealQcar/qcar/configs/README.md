## 如何将外部配置文件传入到观测器中
1. 启动时用 vehicle_main.py --config <carX.yaml>；load_configuration 读取 YAML/JSON → VehicleMainConfig.from_yaml/from_json，现在包含 observer 段（ObserverConfig）。
2. VehicleLogic.__init__ 接收这个 config，取出 config.observer，将其中的 local_estimator_type / fleet_estimator_type 传给 VehicleObserver。
3. VehicleObserverSimple._get_observer_config 会把 config.observer 与默认值合并，支持标量/向量/矩阵的 observer_gain、consensus_gain，生成 self.observer_config。
4. _create_fleet_estimator 把 self.observer_config 中的增益打包进 fleet_config 传给 FleetEstimatorFactory；
5. fleet_state_estimators.py 中的各 estimator 在构造时用这些增益。

## How to transfer the configs into the observer setting
1. tart with vehicle_main.py --config <carX.yaml>; load_configuration reads the YAML/JSON via VehicleMainConfig.from_yaml/from_json, now including the observer block (ObserverConfig).
2. VehicleLogic.__init__ receives this config, pulls out config.observer, and passes its local_estimator_type / fleet_estimator_type into VehicleObserver.
3. VehicleObserverSimple._get_observer_config merges config.observer with defaults, supports scalar/vector/matrix observer_gain and consensus_gain.
4. ._create_fleet_estimator packs the gains from self.observer_config into fleet_config and passes it to FleetEstimatorFactory; 
5. The various estimators in fleet_state_estimators.py use these gains during construction.

## How to use when run the vehicle in the terminal window?

python vehicle_main.py --config configs/car0.yaml