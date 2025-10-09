import numpy as np

class Observer:
    def __init__(self, vehicle, veh_param, initial_global_state, initial_local_state):
        self.vehicle = vehicle
        self.param_sys = veh_param

        self.est_local_state_current = initial_local_state
        self.est_global_state_current = initial_global_state

        num_states = 4
        Nt = self.vehicle.total_time_step
        num_vehicles = len(self.vehicle.other_vehicles)

        self.est_global_state_log = np.zeros((num_states, Nt, num_vehicles))
        self.est_global_state_log[:, 0, :] = initial_global_state

        self.est_local_state_log = np.zeros((num_states, Nt))
        self.est_local_state_log[:, 0] = initial_local_state

        self.tolerances = np.array([5, 2, np.deg2rad(8), 2])
        self.log_element = []
        self.Is_ok_log = []

        if self.vehicle.scenarios_config["Local_observer_type"] == "kalman":
            self.P = np.eye(4)
            self.R = np.diag([0.01, 0.01, 0.0003, 0.01])
            self.Q = np.diag([0.005, 0.005, 0.001, 0.01])
        else:
            self.L_gain = self.place_observer_gain()

    def place_observer_gain(self):
        raise NotImplementedError("Observer gain calculation not implemented.")

    def matrix(self):
        theta = self.vehicle.state[2]
        Ts = self.param_sys["dt"]
        v = self.vehicle.state[3]

        A = np.array([
            [1, 0, -v * np.sin(theta) * Ts, np.cos(theta) * Ts],
            [0, 1,  v * np.cos(theta) * Ts, np.sin(theta) * Ts],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        B = np.array([
            [0, 0],
            [0, 0],
            [0, Ts],
            [Ts, 0]
        ])
        return A, B

    def kalman_filter(self, mesure_state):
        A, B = self.matrix()
        x_pred = A @ self.est_local_state_current + B @ self.vehicle.input
        P_pred = A @ self.P @ A.T + self.Q

        C = np.eye(4)
        y = mesure_state
        S = C @ P_pred @ C.T
        K = P_pred @ C.T @ np.linalg.inv(S)

        self.est_local_state_current = x_pred + K @ (y - C @ x_pred)
        self.P = (np.eye(len(K)) - K @ C) @ P_pred

    def local_observer(self, mesure_state, instant_index):
        if instant_index + 1 >= self.est_local_state_log.shape[1]:
            # Expand the log
            new_cols = 500
            self.est_local_state_log = np.hstack([
                self.est_local_state_log,
                np.zeros((4, new_cols))
            ])
        if self.vehicle.scenarios_config["Is_noise_mesurement"]:
            noise = np.random.multivariate_normal(np.zeros(4), self.R)
            mesure_state += noise

        if self.vehicle.scenarios_config["Local_observer_type"] == "observer":
            A, B = self.matrix()
            self.est_local_state_current = (
                A @ self.est_local_state_current + 
                B @ self.vehicle.input +
                self.L_gain @ (mesure_state - self.est_local_state_current)
            )
        elif self.vehicle.scenarios_config["Local_observer_type"] == "kalman":
            self.kalman_filter(mesure_state)
        else:
            self.est_local_state_current = mesure_state

        self.est_local_state_log[:, instant_index + 1] = self.est_local_state_current

    def check_elementwise_similarity(self, output1, output2, time_idx, vehicle_id):
        diffs = np.abs(output1 - output2)
        is_ok = np.all(diffs <= self.tolerances)
        log_elements = np.where(diffs > self.tolerances)[0].tolist()
        return is_ok, log_elements

    def distributed_observer_each(self, host_id, j, x_bar_j, x_hat_i_j, u_j, weights, use_local=True, predict_only=False):
        A, B = self.matrix()
        Sig = np.zeros(4)

        for L in range(1, len(weights)):
            Sig += weights[L] * (x_hat_i_j[:, L - 1] - x_hat_i_j[:, host_id])

        w_i0 = weights[0]

        if predict_only:
            if use_local and host_id == j:
                return A @ (x_hat_i_j[:, host_id] + w_i0 * (x_bar_j - x_hat_i_j[:, host_id])) + B @ u_j
            else:
                return A @ x_hat_i_j[:, host_id] + B @ u_j
        else:
            if use_local:
                return A @ (x_hat_i_j[:, host_id] + Sig + w_i0 * (x_bar_j - x_hat_i_j[:, host_id])) + B @ u_j
            else:
                if host_id == j:
                    return A @ (x_hat_i_j[:, host_id] + Sig + w_i0 * (x_bar_j - x_hat_i_j[:, host_id])) + B @ u_j
                else:
                    return A @ (x_hat_i_j[:, host_id] + Sig) + B @ u_j

    def distributed_observer(self, instant_index, weights):
        num_vehicles = len(self.vehicle.other_vehicles)
        host_id = self.vehicle.vehicle_number

        Big_X_hat_1_tempo = np.zeros_like(self.est_global_state_current)

        for j in range(num_vehicles):
            x_hat_i_j = np.zeros((4, num_vehicles))
            weights_new = weights.copy()

            if (host_id != j or abs(host_id - j) == 1) and self.vehicle.trip_models[j].flag_local_est_check:
                weights_new[0] = 0

            x_bar_j = self.vehicle.center_communication.get_local_state(j, host_id)
            if x_bar_j is None or np.any(np.isnan(x_bar_j)):
                x_bar_j = np.zeros_like(self.est_local_state_current)
                weights_new[0] = 0

            for k in range(num_vehicles):
                x_hat = self.vehicle.center_communication.get_global_state(k, host_id)
                if x_hat is None or np.any(np.isnan(x_hat)):
                    x_hat = np.zeros_like(self.est_global_state_current)
                    weights_new[k + 1] = 0
                x_hat_i_j[:, k] = x_hat[:, j]

            if self.vehicle.scenarios_config["predict_controller_type"] == "self":
                u_j = self.vehicle.input
            elif self.vehicle.scenarios_config["predict_controller_type"] == "true_other":
                u_j = self.vehicle.other_vehicles[j].input
            else:  # "predict_other"
                if host_id != 1:
                    if j == 1:
                        u_j = np.zeros(2)
                    elif j == host_id:
                        u_j = self.vehicle.input
                    else:
                        est_local_j = self.est_global_state_current[:, j]
                        _, u_j, _ = self.vehicle.controller2.get_optimal_input(
                            j, est_local_j, np.zeros(2),
                            self.vehicle.other_vehicles[j].lane_id, 0,
                            self.vehicle.initial_lane_id,
                            self.vehicle.other_vehicles[j].direction_flag,
                            "est", 0)
                else:
                    u_j = self.vehicle.input

            output = self.distributed_observer_each(host_id, j, x_bar_j, x_hat_i_j, u_j, weights_new, True, False)

            if not self.vehicle.scenarios_config["Use_predict_observer"]:
                Big_X_hat_1_tempo[:, j] = output
                self.Is_ok_log.append(True)
            else:
                if instant_index * self.param_sys["dt"] > 3:
                    output_2 = self.distributed_observer_each(host_id, j, x_bar_j, x_hat_i_j, u_j, weights_new, True, True)
                    is_ok, log_elem = self.check_elementwise_similarity(output, output_2, instant_index, j)
                    self.Is_ok_log.append(is_ok)
                    if is_ok:
                        Big_X_hat_1_tempo[:, j] = output
                    else:
                        self.log_element.append(log_elem)
                        Big_X_hat_1_tempo[:, j] = output_2
                else:
                    Big_X_hat_1_tempo[:, j] = output

        self.est_global_state_current = Big_X_hat_1_tempo
        self.est_global_state_log[:, instant_index, :] = Big_X_hat_1_tempo
