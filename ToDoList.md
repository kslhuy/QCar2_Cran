# To Do List

1. write my own controller strategy
    -vehicle_state_machine.py: 
    throttle, steering, transition = current_state.update(dt, sensor_data)
    in following_leader_state.py & following_path_state
    def _compute_control ()
    write my own controller strategy


- [ ] 找到观测器增益的设置。实现能够为每一辆配置不同的观测器增益 

请再次解释一下，如何将外部配置文件传递至分布式观测器fleet_state-estimator.py。

数据流是：

外部文件（YAML/JSON，例如 configs/carX.yaml）由 vehicle_main.py 加载：VehicleMainConfig.from_yaml/.from_json → 得到 config 对象。
vehicle_logic.py 在 VehicleLogic.__init__ 中把这个 config 传给 VehicleObserver：
obs_cfg = getattr(config, "observer", {})
self.vehicle_observer = VehicleObserver(
    vehicle_id=config.network.car_id,
    config=config,
    logger=self.vehicle_logger,
    local_estimator_type=obs_cfg.get("local_estimator_type", "ekf"),
    fleet_estimator_type=obs_cfg.get("fleet_estimator_type", "consensus"),
)
VehicleObserverSimple._get_observer_config 读取 self.config.observer（若有）并与默认值合并，得到 self.observer_config，其中包含 consensus_gain/observer_gain 等。
创建分布式观测器时，VehicleObserverSimple._create_fleet_estimator 将 self.observer_config 中的增益打包进 fleet_config 传给工厂：
fleet_config = {
    'consensus_gain': self.observer_config.get('consensus_gain', 0.3),
    'observer_gain': self.observer_config.get('observer_gain', 0.1),
}
self.fleet_estimator = FleetEstimatorFactory.create(..., config=fleet_config, ...)
在 fleet_state_estimators.py 内，各个 estimator 构造函数使用 config.get('observer_gain'| 'consensus_gain') 作为实际增益（如分布式 Kalman/ Luenberger 等）。
因此，只要在外部配置的 observer 段写好类型/增益，并用 --config 指向对应文件启动，每辆车的分布式观测器就会拿到各自的配置。