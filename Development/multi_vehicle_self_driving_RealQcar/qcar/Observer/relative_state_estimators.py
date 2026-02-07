"""
Relative State Estimators for Vehicle Observer

Estimates relative states (distance, relative velocity, etc.) between vehicles.
Implements Unknown Input Observers (UIO) and other relative estimation strategies.
"""
import numpy as np
import time
from abc import ABC, abstractmethod
from typing import Optional, Dict, Tuple
import scipy.linalg

# Try importing cvxpy for LMI solving
try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except ImportError:
    CVXPY_AVAILABLE = False


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi)"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

class RelativeStateEstimatorBase(ABC):
    """Base class for all relative state estimators"""
    
    def __init__(self, config: Dict = None, logger=None):
        """
        Initialize base relative state estimator
        
        Args:
            config: Configuration dict
            logger: Logger instance
        """
        self.config = config or {}
        self.logger = logger
        self.state_dim = 4 # Default, e.g., [delta, delta_dot, delta_ddot, f_c]
        
        # Initialize state
        self.state = np.zeros(self.state_dim)
        self.last_update_time = 0.0
        
        # Internal variables for the specific observer implementation
        self.initialized = False

    @abstractmethod
    def update(self, measurement: np.ndarray, dt: float, 
               control_input: Optional[np.ndarray],
               pre_vehicle_state: Optional[np.ndarray] = None,
               host_vehicle_state: Optional[np.ndarray] = None
               ) -> np.ndarray:
        """
        Update relative state estimate
        
        Args:
            measurement: Measurement vector y(k) (e.g., radar/camera data + communication)
            dt: Time step
            control_input: Control input (e.g., acceleration)
            pre_vehicle_state: State of preceding vehicle (if available/needed for input)
            host_vehicle_state: State of host vehicle (if available/needed for input)
            
        Returns:
            Updated state estimate
        """
        pass
    
    @abstractmethod
    def get_state(self) -> np.ndarray:
        """Get current state estimate"""
        pass
    
    @abstractmethod
    def reset(self):
        """Reset estimator"""
        pass

    def add_received_local_state(self, sender_id: int, state: Dict, timestamp_ns: int) -> bool:
        """Add a received LOCAL state (dict or ndarray) and store ndarray in history.

        Communication/log layers hand us dicts; algorithms want numpy arrays.
        We normalize here so downstream consumers always see ndarray.
        """
        # self.logger.logger.debug(f"Adding received local state from vehicle_id {sender_id}")
        try:
            if sender_id == self.vehicle_id:
                return False  # Don't store own state

            state_vec: Optional[np.ndarray] = None
            if isinstance(state, dict):
                state_vec = _state_dict_to_array(state, self.state_dim, logger=self.logger)
            else:
                state_vec = _normalize_state_array(state, self.state_dim, logger=self.logger)

            if state_vec is None:
                return False

            # Store timestamp in nanoseconds (keep ndarray only)
            self.received_local_states[sender_id].append((timestamp_ns, state_vec.copy()))

            # Keep only recent history (default 10 entries)
            if len(self.received_local_states[sender_id]) > 10:
                self.received_local_states[sender_id] = self.received_local_states[sender_id][-10:]

            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received local state error", e)
            return False

    def add_received_fleet_state(self, sender_id: int, fleet_estimates: Dict, timestamp_ns: int) -> bool:
        """Add a received FLEET state broadcast from another vehicle.

        Default implementation stores the raw fleet dictionary in
        `received_fleet_states` and also unpacks per-vehicle states into
        `received_states` for easy access by other algorithms.
        """
        try:
            if sender_id == self.vehicle_id:
                return False

            # Check for new vehicle IDs and expand capacity if required
            try:
                max_id_in_msg = max((int(vid) for vid in fleet_estimates.keys()), default=0)
            except Exception:
                max_id_in_msg = 0

            if max_id_in_msg >= self.fleet_size:
                self._ensure_fleet_capacity(max_id_in_msg)

            # Store the raw fleet dictionary with timestamp
            self.received_fleet_states[sender_id].append((timestamp_ns, fleet_estimates))


            # Limit history for fleet snapshots per neighbor (default 5)
            if len(self.received_fleet_states[sender_id]) > 5:
                self.received_fleet_states[sender_id] = self.received_fleet_states[sender_id][-5:]

            return True

        except Exception as e:
            if self.logger:
                self.logger.log_error("Add received fleet state error", e)
            return False

    def _get_latest_fleet_data(self, neighbor_id: int, current_time_ns: int) -> Optional[Dict]:
        """Return the newest fleet dictionary from a neighbor that is still valid.

        Returns None if there is no recent valid snapshot.
        """
        if neighbor_id not in self.received_fleet_states:
            return None

        history = self.received_fleet_states[neighbor_id]
        if not history:
            return None

        # Iterate backwards to find newest valid data
        for ts_ns, fleet_data in reversed(history):
            if (current_time_ns - ts_ns) <= self.max_state_age_ns:
                return fleet_data
        return None

    def _get_latest_received_state(self, vehicle_id: int, current_time_ns: int) -> Optional[np.ndarray]:
        """Return the most recent received state (as ndarray) within the age limit.

        Returns None if no recent state is available or conversion fails.
        """
        if vehicle_id not in self.received_local_states:
            if self.logger:
                self.logger.logger.warning(f"vehicle_id {vehicle_id} not in self.received_local_states")
            return None

        states_list = self.received_local_states[vehicle_id]
        if not states_list:
            if self.logger:
                self.logger.logger.warning(f"states_list for vehicle_id {vehicle_id} is empty")
            return None

        # Iterate backwards (newest first) and return first valid entry
        for timestamp_ns, state in reversed(states_list):
            age_ns = current_time_ns - timestamp_ns
            if age_ns > self.max_state_age_ns:
                continue

            # History stores ndarray; older entries might still be dict, so normalize defensively
            if isinstance(state, dict):
                state_vec = _state_dict_to_array(state, self.state_dim, logger=self.logger)
            else:
                state_vec = _normalize_state_array(state, self.state_dim, logger=self.logger)

            if state_vec is not None:
                return state_vec
            
        # if self.logger:
        #     self.logger.logger.debug(f"No valid recent state for vehicle_id {vehicle_id}")
        return None



    def _compute_relative_measurement_from_v2v(self, host_state: np.ndarray, pre_state: np.ndarray) -> np.ndarray:
        """
        Compute relative measurement (distance, relative velocity) from absolute states.
        
        Args:
            host_state: Host vehicle state [x, y, theta, v, ...]
            pre_state: Preceding vehicle state [x, y, theta, v, ...]
            
        Returns:
            np.ndarray: Derived measurement vector [delta, delta_dot]
        """
        if host_state is None or pre_state is None:
            if self.logger:
                 self.logger.logger.warning("Cannot compute relative measurement: missing vehicle states.")
            return np.zeros(2)

        # Extract positions and velocities
        # Assuming state structure: [x, y, theta, v, a] (5 elements)
        # We need relative distance along path or Euclidean.
        
        dx = pre_state[0] - host_state[0]
        dy = pre_state[1] - host_state[1]
        
        dist = np.sqrt(dx**2 + dy**2)
        
        # Relative velocity
        # delta_dot = v_pre - v_host
        v_host = host_state[3]
        v_pre = pre_state[3]
        
        delta_dot = v_pre - v_host
        
        return np.array([dist, delta_dot])


class SA_ACC_UIO_Estimator(RelativeStateEstimatorBase):
    """
    Secure Aware ACC Unknown Input Observer (SA_ACC_UIO)
    
    Based on SA_ACC_UIO.m MATLAB implementation.
    Estimates relative distance, velocity, acceleration, and cyber-attack/disturbance.
    Uses LMI to compute gains if CVXPY is available.
    """
    
    def __init__(self, config: Dict = None, logger=None):
        super().__init__(config, logger)
        
        # System parameters (defaults from MATLAB script)
        self.Ts = self.config.get('Ts', 0.01)
        self.tau = self.config.get('tau', 0.4)
        self.li = self.config.get('li', 5.0)
        self.Li = self.config.get('Li', 8.0)
        self.h = self.config.get('h', 0.5)
        self.k1 = self.config.get('k1', -0.8)
        self.k2 = self.config.get('k2', 2.5)
        
        # Derived parameter k3
        self.k3 = self.config.get('k3', (1 - self.h * self.k1 * self.k2))
        
        # Observer LMI alpha
        self.alpha_lmi = self.config.get('alpha_lmi', 0.5)

        # State dimensions
        # State: [delta_i, delta_i_dot, delta_i_ddot, f_c]
        # But MATLAB script uses matrices that imply a specific state order.
        # checking MATLAB: E = [eye(length(Ad)) Wd]; length(Ad) is 3. Wd is 3x1. 
        # So Extended system state (descriptor) is likely [psi; f_c] -> 4 states.
        # psi = [delta_i, delta_i_dot, delta_i_ddot]
        self.state_dim = 4 
        
        # Internal state vector w (auxiliary state for UIO)
        self.w = None 
        
        # Initialize Matrices
        self._initialize_matrices()
        
        # Compute Gains
        self._compute_gains()
        
        self.initialized = True
        
    def _initialize_matrices(self):
        """Parameterize and discretize the system matrices"""
        # Continuous-time SA-ACC system matrices (from MATLAB)
        # Ac = [0 1 0; 0 0 1; -k2/(tau*h) -k3/(tau*h) (k1-k3)/(tau)]
        Ac = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [-self.k2/(self.tau*self.h), -self.k3/(self.tau*self.h), (self.k1 - self.k3)/self.tau]
        ])
        
        # Bc = [0; 0; k2/tau]
        Bc = np.array([[0], [0], [self.k2/self.tau]])
        
        # Fc = [0; 0; k3/tau]
        Fc = np.array([[0], [0], [self.k3/self.tau]])
        
        # Wc = [0; 0; (k3-k1)/tau]
        Wc = np.array([[0], [0], [(self.k3 - self.k1)/self.tau]])
        
        # deltac = [0; 0; k2*(Li-li)/(tau*h)]
        self.deltac = np.array([[0], [0], [self.k2*(self.Li - self.li)/(self.tau*self.h)]])
        
        # Cc = [1 0 0; 0 1 0]
        Cc = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])
        
        # Discretization
        # Ad = (eye + Ts*Ac)
        self.Ad = np.eye(3) + self.Ts * Ac
        self.Bd = self.Ts * Bc
        self.Fd = self.Ts * Fc
        self.Wd = self.Ts * Wc
        self.deltad = self.Ts * self.deltac
        self.Cd = Cc
        
        # Descriptor form setup
        # E = [eye(length(Ad)) Wd] -> [I_3  Wd] (3x4)
        self.E = np.hstack([np.eye(3), self.Wd])
        
        # Ae = [Ad zeros(3,1)] (3x4)
        self.Ae = np.hstack([self.Ad, np.zeros((3, 1))])
        
        # Ce = [Cd*Ad zeros(2,1)] (2x4)
        self.Ce = np.hstack([self.Cd @ self.Ad, np.zeros((2, 1))])
        
        # Constants vectors from MATLAB
        self.E_w = np.array([[1], [0], [0]]) # 3x1
        self.D_w = np.array([[1], [0]])      # 2x1
        
    def _compute_gains(self):
        """Compute observer gains using LMI if possible"""
        # 1. Compute Pz and Qz using Generalized Inverse approach
        # g_invEC = inv([E; Ce]' * [E; Ce]) * [E; Ce]'
        # [E; Ce] is (3+2) x 4 = 5x4 matrix.
        
        mat_EC = np.vstack([self.E, self.Ce]) # 5x4
        
        # Pseudo-inverse is safer than performing inv(M.T @ M) @ M.T explicitly if ill-conditioned,
        # but MATLAB code does the explicit formula. We'll use pinv for stability equivalent.
        g_invEC = np.linalg.pinv(mat_EC) # 4x5
        
        # Pz = g_invEC(1:4, 1:3) -> Top 3 columns (indices 0,1,2 in Python)
        # Note: MATLAB 1:3 is columns 1,2,3. Python 0:3.
        self.Pz = g_invEC[:, 0:3] # 4x3
        
        # Qz = g_invEC(:, 4:5) -> Columns 4,5 (indices 3,4 in Python)
        self.Qz = g_invEC[:, 3:5] # 4x2
        
        # M = Pz * Be (Be = Bd)
        self.M = self.Pz @ self.Bd
        
        # G = Pz * Fe (Fe = Fd)
        self.G = self.Pz @ self.Fd
        
        # Pz * deltae (deltae = deltad)
        self.Pz_deltad = self.Pz @ self.deltad
        
        # LMI Design Parameters
        # Az = Pz * Ae
        self.Az = self.Pz @ self.Ae # 4x4
        
        self.Dz = self.Qz @ self.D_w
        self.xi_z = self.Pz @ self.E_w
        
        # Solve LMI for K
        self.K = np.zeros((4, 2)) # Default K
        
        # Check config for manual K
        if 'K' in self.config:
            self.K = np.array(self.config['K'])
            if self.logger:
                self.logger.logger.info("SA_ACC_UIO: Using manually configured gain K.")
        elif CVXPY_AVAILABLE:
            try:
                if self.logger:
                    self.logger.logger.info("SA_ACC_UIO: Solving LMI with CVXPY...")
                
                nx = 4 # State dimension of observer state w (which estimates full state xi?)
                # Wait, in MATLAB: L = K + N*Qz. w is updated. 
                # ksi_hat = w + Qz*y. ksi_hat is 4x1. So w is 4x1. Az is 4x4.
                
                # Variables
                P = cp.Variable((nx, nx), PSD=True) # Symmetric Positive Semi-Definite
                Z = cp.Variable((2, nx)) # Z = X' in MATLAB code? 
                # MATLAB: Z = sdpvar(p, nx); p=2. 
                # But MATLAB LMI uses Xs'. Let's follow MATLAB exactly.
                # K = inv(Ps)*Xs' -> K = P_inv * Z.T -> So Z must be (nx, p)?
                # MATLAB: Z = sdpvar(p, nx) -> 2x4. 
                # Xs = double(Z). 
                # K = inv(Ps) * Xs'. (4x4) * (2x4)' = 4x2. Correct.
                # So Z in my code (to match python shape conventions usually Z is K*P)
                # Let's stick to MATLAB variable shapes to avoid confusion.
                Z_mat = cp.Variable((2, nx)) # 2x4
                S = cp.Variable((2, 2), PSD=True)
                
                # Terms
                # sup_LMI = [(D_w'*Z - xi_z'*P); D_z'*P];
                # D_w is 2x1. Z is 2x4. D_w'*Z is 1x4.
                # xi_z is 4x1. P is 4x4. xi_z'*P is 1x4.
                # Row 1 of sup_LMI: (1x4)
                # D_z is 4x1? No Dz = Qz * Dw. Qz is 4x2, Dw is 2x1. -> Dz is 4x1.
                # D_z'*P is 1x4.
                # sup_LMI is 2x4.
                
                term1 = self.D_w.T @ Z_mat - self.xi_z.T @ P
                term2 = self.Dz.T @ P
                sup_LMI = cp.vstack([term1, term2]) # 2x4
                
                # Matrix LMI
                # [-alpha*P             , zeros(4,2)         , Az'*P - Ce'*Z;
                #   zeros(4,2)'          ,-S         , sup_LMI   
                #   (Az'*P - Ce'*Z)'   ,sup_LMI'   , -P];
                
                # Block 1,3: Az'*P - Ce'*Z_mat -> (4x4)(4x4) - (2x4)'(2x4) -> 4x4 - 4x4?
                # Ce is 2x4. Z_mat is 2x4. Ce'*Z_mat is 4x2 * 2x4? No, MATLAB Z is p x nx (2x4).
                # Term is Az'*P - Ce'*Z. 
                # In python cp: Az.T @ P - Ce.T @ Z_mat. 
                # Shape: (4x4) - (4x2)(2x4) = 4x4. Correct.
                
                block13 = self.Az.T @ P - self.Ce.T @ Z_mat
                
                # Construct big matrix
                # Row 1
                row1 = cp.hstack([-self.alpha_lmi * P, np.zeros((4, 2)), block13])
                # Row 2
                row2 = cp.hstack([np.zeros((2, 4)), -S, sup_LMI])
                # Row 3
                row3 = cp.hstack([block13.T, sup_LMI.T, -P])
                
                LMI_mat = cp.vstack([row1, row2, row3])
                
                constraints = [LMI_mat <= 0, P >= 0, S >= 0] # P>=0, S>=0 redundancy usually fine
                
                prob = cp.Problem(cp.Minimize(0), constraints)
                prob.solve()
                
                if prob.status == cp.OPTIMAL:
                    P_val = P.value
                    Z_val = Z_mat.value
                    
                    # Compute K = inv(P) * Z'
                    # Regularize P inverse if needed
                    self.K = np.linalg.solve(P_val, Z_val.T) # 4x2
                    if self.logger:
                        self.logger.logger.info("SA_ACC_UIO: LMI solved successfully. K computed.")
                else:
                    if self.logger:
                        self.logger.log_warning(f"SA_ACC_UIO: LMI solver failed. Status: {prob.status}")
                        
            except Exception as e:
                if self.logger:
                    self.logger.log_error("SA_ACC_UIO: CVXPY error", e)
        else:
            if self.logger:
                self.logger.log_warning("SA_ACC_UIO: CVXPY not available. Using fallback/zero gains.")
        
        # Compute final observer matrices
        # N = Az - K * Ce
        self.N = self.Az - self.K @ self.Ce
        
        # L = K + N * Qz
        self.L = self.K + self.N @ self.Qz
        
        # Initialize w state (4x1)
        self.w = np.zeros(4)

    def update(self, measurement: np.ndarray, dt: float, 
               control_input: Optional[np.ndarray],
               pre_vehicle_state: Optional[np.ndarray] = None,
               host_vehicle_state: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Update the UIO observer.
        
        Args:
            measurement: y(k) vector. Expects 2 elements: [y1, y2]
                         Typically relative distance/velocity or similar.
                         MATLAB: y(:,i) = Cd*psi(:,i) + ... -> 2 outputs.
            dt: Time step (unused if fixed Ts, but kept for interface)
            pre_vehicle_state: Not explicitly used in update equation W(k+1) except via input?
            host_vehicle_state: xf(2, i-1) used in MATLAB (velocity of host).
            
        MATLAB Update:
           ksi_hat(:,i) = w(:,i) + Qz*y(:,i)
           w(:,i+1) = N*w(:,i) + L*y(:,i) + M*xf(2,i) + G*mu(i) + Pz*deltae
           
           Wait, mu(i) is unknown input? Or known?
           In MATLAB simulation: mu(i) = ai_1(i) + fc(i). 
           But the observer equations use G*mu(i-1)? 
           Actually, UIO is designed to estimate unknown input, but if 'mu' is passed 
           in line 173: `M*xf(2,i-1) + G*mu(i-1)`, it implies mu is KNOWN input?
           
           Re-reading MATLAB:
           System: E*x(k+1) = Ae*x(k) + Be*u(k) + Fe*d(k) ...
           Be = Bd (input matrix for host velocity xf(2)). 
           Fe = Fd.
           
           In MATLAB `w(:,i+1) = ... G*mu(i-1) ...`
           If mu is the attack + acceleration, we typically don't know it.
           Usually UIO decouples unknown input.
           
           Let's look at `Fe`. Fd = Ts*Fc. Fc = [0;0;k3/tau].
           This term relates to `mu`.
           
           If this is an Unknown Input Observer, we should not need `mu`.
           However, in the MATLAB code line 173: `G*mu(i-1)`.
           This implies this specific implementation treats `mu` as a KNOWN input in that simulation context?
           OR `G` matrix is zero?
           G = Pz * Fe.
           If unknown input is decoupled, then Pz*Fe should be 0.
           Let's check `g_invEC` definition. `g_invEC * [E; Ce]' = I`.
           This decouples E.
           The standard UIO condition is to decouple the unknown input direction.
           
           If `mu` is unknown, we cannot use it in update.
           I will assume `mu` is 0 or ignored in the observer update if it's truly unknown, 
           OR the python implementation needs to accept it if it's known (e.g. communicated acceleration).
           
           In SA-ACC, `mu` is often the preceding vehicle acceleration (via V2V). 
           So it might be treated as known input, and `fc` (attack) is the unknown part?
           The matrix `Wd` is for `fc`.
           E = [I Wd]. The "unknown" part being estimated/decoupled is related to `Wd` (the attack/disturbance).
           `mu` (preceding acceleration) is treated as KNOWN (from V2V).
           
           So arguments needed:
           - measurement (y)
           - host_velocity (u_host, xf(2))
           - preceding_acceleration (u_pre, mu) (Comm. acc)
        """
        
        # Parse inputs
        # y: 2 Dim [delta, delta_dot] measured?
        y_k = measurement
        
        # Check if we should fake the measurement using V2V data
        use_fake_meas = self.config.get('use_fake_relative_measurements', False)
        
        if use_fake_meas:
            if host_vehicle_state is not None and pre_vehicle_state is not None:
                y_k = self._compute_relative_measurement_from_v2v(host_vehicle_state, pre_vehicle_state)
                # if self.logger:
                #     self.logger.logger.debug(f"Using fake relative measurement: {y_k}")
            else:
                if self.logger:
                    self.logger.logger.warning("use_fake_relative_measurements=True but missing states. Using raw measurement.")

        if y_k.shape[0] != 2:
            # Try to grab relevant parts if larger
            pass

        
        # Inputs
        # Host velocity (xf(2))
        u_host_vel = 0.0
        if host_vehicle_state is not None:
            # Assuming state [x, y, theta, v, a]
            u_host_vel = host_vehicle_state[3]
             
        # Preceding acceleration (mu)
        u_pre_acc = 0.0
        if pre_vehicle_state is not None:
            # Assuming state [x, y, theta, v, a]
            u_pre_acc = pre_vehicle_state[4]
             
        # 1. Estimation Step: ksi_hat = w + Qz*y
        ksi_hat = self.w + self.Qz @ y_k
        
        # 2. Update auxiliary state w for NEXT step
        # w(k+1) = N*w(k) + L*y(k) + M*u_host + G*u_pre + Pz*deltae
        
        # M is 4x1? Bd is 3x1. Pz is 4x3. M = Pz*Bd -> 4x1.
        # u_host_vel is scalar.
        term_M = self.M.flatten() * u_host_vel
        
        # G = Pz*Fd. Fd is 3x1. G is 4x1.
        term_G = self.G.flatten() * u_pre_acc
        
        term_Pz_delta = self.Pz_deltad.flatten()
        
        # Update w
        self.w = (self.N @ self.w) + (self.L @ y_k) + term_M + term_G + term_Pz_delta
        
        # Store state
        self.state = ksi_hat
        self.last_update_time = time.time()
        
        return ksi_hat

    def get_state(self) -> np.ndarray:
        return self.state.copy()

    def reset(self):
        self.w = np.zeros(4)
        self.state = np.zeros(4)

class RelativeEstimatorFactory:
    """Factory for relative state estimators"""
    
    @staticmethod
    def create(estimator_type: str, config: Dict = None, logger=None) -> RelativeStateEstimatorBase:
        if estimator_type == 'sa_acc_uio':
            return SA_ACC_UIO_Estimator(config, logger)
        else:
            raise ValueError(f"Unknown relative estimator type: {estimator_type}")
