import math

class ControllerFollower:
    def __init__(self, qcar, trust_model, k_steering=2.0, max_steering=0.6,
                 max_speed=1.0, safe_distance=0.5):
        self.qcar = qcar
        self.trust_model = trust_model
        self.k_steering = k_steering
        self.max_steering = max_steering
        self.max_speed = max_speed
        self.default_safe_distance = safe_distance
        self.leader_state = None  # To be set externally

    @staticmethod
    def wrap_to_pi(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def update_leader_state(self, state):
        self.leader_state = state

    def step(self):
        if self.leader_state is None:
            return  # wait until data is received

        pos_leader = self.leader_state['position']
        rot_leader = self.leader_state['rotation']

        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()

        # Project leader forward
        lookahead_distance = 0.4
        target_x = pos_leader[0] - lookahead_distance * math.cos(rot_leader[2])
        target_y = pos_leader[1] - lookahead_distance * math.sin(rot_leader[2])

        dx = target_x - pos_follower[0]
        dy = target_y - pos_follower[1]
        distance = math.hypot(dx, dy)

        follower_heading = rot_follower[2]
        target_angle = math.atan2(dy, dx)
        heading_error = self.wrap_to_pi(target_angle - follower_heading)

        # Steering
        steering_cmd = -self.k_steering * heading_error
        steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

        # Estimate velocities
        v_follower = getattr(self.qcar, 'motorTach', 0.5)
        v_leader = self.leader_state.get('velocity', 0.5)
        a_leader = self.leader_state.get('acceleration', 0.0)
        b_leader = 0.2  # Assume recent timestamp

        # Trust evaluation
        v_score = self.trust_model.evaluate_velocity(
            host_id=1, target_id=0,
            v_y=v_follower, v_host=v_follower,
            v_leader=v_leader, a_leader=a_leader, b_leader=b_leader
        )
        self.trust_model.update_rating_vector(v_score, rating_type="local")
        # trust_score = self.trust_model.calculate_trust_score(self.trust_model.rating_vector)
        trust_score = 1
        adjusted_distance = self.trust_model.determine_following_distance(
            trust_score, ds=self.default_safe_distance, dacc=2.0
        )

        # if distance < (adjusted_distance - 0.05):  # 5 cm margin
        #     speed_cmd = 0.0
        # else:
        #     # speed_cmd = min(self.max_speed, 0.5 * (distance - adjusted_distance))
        speed_cmd = max(0.0, min(self.max_speed, 0.5 * (distance - adjusted_distance)))


        # if distance < adjusted_distance:
        #     speed_cmd = 0.0
        # else:
        #     speed_cmd = min(self.max_speed, 0.5 * (distance - adjusted_distance))

        print(f"[Trust] Score: {trust_score:.2f} | Dist: {distance:.2f} | Target: {adjusted_distance:.2f} | Speed: {speed_cmd:.2f}")

        # Send to QCar
        try:
            self.qcar.set_velocity_and_request_state(
                forward=speed_cmd,
                turn=steering_cmd,
                headlights=False,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=False,
                reverseSignal=False
            )
        except Exception as e:
            print("[ControllerFollower] Command failed:", e)
