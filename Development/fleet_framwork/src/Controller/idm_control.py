import math

class IDMControl:
    def __init__(self, controller):
        self.controller = controller
        self.param_opt = controller.param_opt
        self.param_sys = controller.param_sys
        self.goal = controller.goal
        self.straightlane = controller.straightlane
        self.vehicle_id = controller.vehicle_id

    def get_optimal_input(self, host_car_id, state, last_input, lane_id,
                          input_log, initial_lane_id, direction_flag, type_state, acc_flag):
        acc_flag = 0

        # Extract parameters
        alpha = self.param_opt['alpha']
        beta = self.param_opt['beta']
        v0 = self.param_opt['v0']
        delta_exp = self.param_opt['delta']
        T = self.param_opt['T']
        s0 = self.param_opt['s0']

        # Unpack current state
        x, y, theta, v = self.unpack_state(state)

        # Get leading vehicle info
        _, car_fss, _, _ = self.controller.get_surrounding_vehicles(
            x, lane_id, direction_flag, host_car_id)
        car_font = car_fss[-1] if car_fss else None

        if car_font is None:
            print("No car ahead")
            print(self.vehicle_id)
            s = 200  # default large gap
            delta_v = 0
        else:
            s = math.hypot(car_font.state[0] - x, car_font.state[1] - y)
            delta_v = v - car_font.state[3]

        # Desired minimum gap
        s_star = s0 + T * v + (v * delta_v) / (2 * (alpha * beta)**0.5)

        # IDM acceleration
        acc = alpha * (1 - (v / v0)**delta_exp - (s_star / s)**2)

        # Simple constant steering model
        delta = 0

        input_u = [acc, delta]
        e = 0  # error placeholder
        return acc_flag, input_u, e

    @staticmethod
    def unpack_state(state):
        x, y, theta, v = state
        return x, y, theta, v

    @staticmethod
    def compute_curvature(v, theta):
        return theta / max(v, 0.1)

    @staticmethod
    def compute_s_bar(kappa, ri, hi, v):
        if kappa == 0:
            return 0
        return (-1 + (1 + kappa**2 * (ri + hi * v)**2)**0.5) / kappa
