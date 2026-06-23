import numpy as np
import math


from Observer.Obs_6d_on_track_sysID.vehicle_dynamics_stown import vehicle_dynamics_st


class ParamsDummy:
    """
    Dummy class for parameters, user will need to replace this with the
    actual vehicle parameter class used in the project configuration.
    """

    def __init__(self):
        self.m = 2.56  # mass [kg]
        self.I_z = 0.038  # yaw moment of inertia [kg*m^2]
        self.l_f = 0.14  # distance CG to front axle [m]
        self.l_r = 0.14  # distance CG to rear axle [m]
        self.h_cg = 0.12  # CG height [m]

        # linear tire model params
        self.C_Sf = 50.0
        self.C_Sr = 50.0

        # pacejka tire model params (B, C, D, E)
        self.C_Pf = [10.0, 1.9, 1.0, 0.97]
        self.C_Pr = [10.0, 1.9, 1.0, 0.97]


class LateralVelocityEKF:
    def __init__(self, p=None, dt=0.01, tire_model_type="linear"):
        """
        EKF for estimating lateral velocity (v_y) and yaw rate (omega).

        Args:
            p: Vehicle parameter object (defaults to DummyParams)
            dt: Time step
            tire_model_type: "linear" or "pacejka" for tire model
        """
        self.p = p if p is not None else ParamsDummy()
        self.dt = dt
        self.type = tire_model_type

        # State vector corresponds exactly to vehicle_dynamics_st inputs:
        # X = [x, y, psi, v_x, v_y, omega]
        self.x_state = np.zeros(6)

        # Covariance matrix P
        self.P = np.eye(6) * 1.0

        # Process noise covariance Q
        # We model high confidence in our knowledge of x, y, psi, vx
        # and lower confidence in the physics model for v_y and omega
        self.Q = np.diag([1e-6, 1e-6, 1e-6, 0.01, 1.0, 0.5])

        # Measurement noise covariance R for [omega_meas, a_y_meas, v_x_meas]
        self.R_imu = np.diag([0.01, 1.0, 0.01])

        # Measurement noise covariance R for [omega_meas, a_y_meas, v_x_meas, x, y, psi]
        self.R_gps = np.diag([0.01, 1.0, 0.01, 0.04, 0.04, 0.0004])

    def _f_sys(self, x, u):
        """Wrapper for the system dynamics"""
        # vehicle_dynamics_st returns f = [x_dot, y_dot, psi_dot, vx_dot, vy_dot, omega_dot]
        return np.array(vehicle_dynamics_st(x, u, self.p, self.type))

    def predict(self, delta, ax, vx_meas=None):
        """
        Predict step of the EKF.

        Args:
            delta: front wheel steering angle [rad]
            ax: longitudinal acceleration [m/s^2]
            vx_meas: highly accurate measured longitudinal velocity (optional).
                     If provided, overrides the model's internal v_x.
        """
        # u = [steering_angle, long_acceleration]
        u = np.array([delta, ax])

        if vx_meas is not None:
            # Overwrite v_x state with measured value if available
            # since v_x is usually accurately measured by motor tachometers
            self.x_state[3] = vx_meas

        # RK4 Integration for the state prediction
        k1 = self._f_sys(self.x_state, u)
        k2 = self._f_sys(self.x_state + 0.5 * self.dt * k1, u)
        k3 = self._f_sys(self.x_state + 0.5 * self.dt * k2, u)
        k4 = self._f_sys(self.x_state + self.dt * k3, u)

        x_pred = self.x_state + (self.dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        # Compute State Jacobian F = I + dt * df/dx numerically
        F = np.eye(6)
        eps = 1e-5
        for i in range(6):
            x_pert = self.x_state.copy()
            x_pert[i] += eps
            f_pert = self._f_sys(x_pert, u)
            df_dx_i = (f_pert - k1) / eps
            F[:, i] += self.dt * df_dx_i

        # Update predicted state and covariance
        self.x_state = x_pred
        self.P = F @ self.P @ F.T + self.Q

    def _unwrap_angle(self, angle: float, reference: float) -> float:
        """Unwrap angle to be continuous with reference."""
        diff = angle - reference
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return reference + diff

    def _h_meas(self, x, u, use_gps=False):
        """
        Measurement equations that map state to sensors.
        If use_gps: z_pred = [omega, a_y, v_x, x, y, psi]
        Else: z_pred = [omega, a_y, v_x]
        """
        f = self._f_sys(x, u)
        omega = x[5]  # state index 5 is omega
        v_y_dot = f[4]  # index 4 is vy_dot
        v_x = x[3]  # index 3 is v_x

        # Lateral acceleration from IMU is a_y = v_y_dot + v_x * omega
        a_y = v_y_dot + v_x * omega

        if use_gps:
            return np.array([omega, a_y, v_x, x[0], x[1], x[2]])
        return np.array([omega, a_y, v_x])

    def update(self, omega_meas, ay_meas, delta, ax, vx_meas, gps_data=None):
        """
        Update step of the EKF.

        Args:
            omega_meas: measured yaw rate from gyro [rad/s]
            ay_meas: measured lateral acceleration from accelerometer [m/s^2]
            delta: front steering angle input
            ax: longitudinal acceleration input
            vx_meas: measured longitudinal velocity [m/s]
            gps_data: optional dictionary with 'x', 'y', 'psi'

        Returns:
            v_y: the new laterally estimated velocity
        """
        u = np.array([delta, ax])

        use_gps = gps_data is not None

        if use_gps:
            meas_psi = self._unwrap_angle(gps_data["psi"], self.x_state[2])
            z_meas = np.array(
                [omega_meas, ay_meas, vx_meas, gps_data["x"], gps_data["y"], meas_psi]
            )
            R = self.R_gps
            meas_dim = 6
        else:
            z_meas = np.array([omega_meas, ay_meas, vx_meas])
            R = self.R_imu
            meas_dim = 3

        # Predicted measurement
        z_pred = self._h_meas(self.x_state, u, use_gps)

        # Measurement Jacobian H numerically
        H = np.zeros((meas_dim, 6))
        eps = 1e-5
        for i in range(6):
            x_pert = self.x_state.copy()
            x_pert[i] += eps
            z_pert = self._h_meas(x_pert, u, use_gps)
            H[:, i] = (z_pert - z_pred) / eps

        # Innovation (measurement residual)
        y = z_meas - z_pred

        if use_gps:
            y[5] = self._unwrap_angle(y[5], 0.0)

        # Kalman Gain
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.inv(S + np.eye(meas_dim) * 1e-6)

        K = self.P @ H.T @ S_inv

        # Update state and covariance
        self.x_state = self.x_state + K @ y
        self.x_state[2] = self._unwrap_angle(self.x_state[2], 0.0)
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P

        # Return estimated v_y
        return self.x_state[4]
