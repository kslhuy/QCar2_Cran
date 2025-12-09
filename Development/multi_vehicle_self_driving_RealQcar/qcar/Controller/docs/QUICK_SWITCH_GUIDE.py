"""
QUICK SWITCH GUIDE - How to Change Controllers
==============================================

This file shows the EXACT steps to switch between controllers.
Copy-paste the code you need!
"""

# ============================================================
# METHOD 1: Direct code change (fastest for testing)
# ============================================================

# In following_leader_state.py, line ~34:

# Option A: Use CACC (spacing + velocity control)
self.controller_type = 'cacc'

# Option B: Use PI (simple velocity tracking)
self.controller_type = 'pi'

# Option C: Use Hybrid (automatic switching)
self.controller_type = 'hybrid'


# ============================================================
# METHOD 2: Custom parameters (for fine-tuning)
# ============================================================

# In following_leader_state.py, inside _initialize_longitudinal_controller():

# Conservative CACC (safe, larger spacing)
if self.controller_type == 'cacc':
    params = {
        's0': 2.0,      # ← Larger minimum spacing
        'h': 0.7,       # ← Longer time headway
        'K': np.array([[0.15, 0.03]]),  # ← Gentler gains
        'acc_to_throttle_gain': 0.4,
        'max_throttle': 0.25
    }

# Aggressive CACC (tight, fast response)
if self.controller_type == 'cacc':
    params = {
        's0': 1.0,      # ← Smaller spacing
        'h': 0.3,       # ← Shorter time headway
        'K': np.array([[0.3, 0.08]]),  # ← Higher gains
        'acc_to_throttle_gain': 0.6,
        'max_throttle': 0.3
    }

# Fast PI (quick response)
if self.controller_type == 'pi':
    params = {
        'kp': 0.2,      # ← Higher proportional gain
        'ki': 1.5,      # ← Higher integral gain
        'max_throttle': 0.3
    }

# Smooth PI (gentle response)
if self.controller_type == 'pi':
    params = {
        'kp': 0.05,     # ← Lower proportional gain
        'ki': 0.5,      # ← Lower integral gain
        'max_throttle': 0.25
    }


# ============================================================
# METHOD 3: Configuration-based (recommended for production)
# ============================================================

# Step 1: Add to your config class (e.g., VehicleConfig)
class VehicleConfig:
    # ... existing config ...
    
    # Controller selection
    longitudinal_controller_type = 'cacc'  # 'cacc', 'pi', or 'hybrid'
    
    # CACC parameters
    cacc_s0 = 1.5
    cacc_h = 0.5
    cacc_k_spacing = 0.2
    cacc_k_velocity = 0.05
    cacc_acc_gain = 0.5
    cacc_max_throttle = 0.3
    
    # PI parameters
    pi_kp = 0.1
    pi_ki = 1.0
    pi_max_throttle = 0.3

# Step 2: Use config in following_leader_state.py
def _initialize_longitudinal_controller(self):
    # Read from config
    controller_type = getattr(self.config, 'longitudinal_controller_type', 'cacc')
    
    if controller_type == 'cacc':
        params = {
            's0': getattr(self.config, 'cacc_s0', 1.5),
            'h': getattr(self.config, 'cacc_h', 0.5),
            'K': np.array([[
                getattr(self.config, 'cacc_k_spacing', 0.2),
                getattr(self.config, 'cacc_k_velocity', 0.05)
            ]]),
            'acc_to_throttle_gain': getattr(self.config, 'cacc_acc_gain', 0.5),
            'max_throttle': getattr(self.config, 'cacc_max_throttle', 0.3)
        }
    # ... etc


# ============================================================
# METHOD 4: Runtime switching (advanced)
# ============================================================

# Add this method to FollowingLeaderState class:
def switch_controller(self, new_type: str, params: dict = None):
    """
    Switch to a different controller at runtime
    
    Args:
        new_type: 'cacc', 'pi', or 'hybrid'
        params: Optional custom parameters
    """
    self.logger.logger.info(f"Switching from {self.controller_type} to {new_type}")
    
    # Reset current controller
    if self.longitudinal_controller:
        self.longitudinal_controller.reset()
    
    # Update type
    self.controller_type = new_type
    
    # Reinitialize with new type
    if params:
        self.longitudinal_controller = ControllerFactory.create(
            new_type, params, logger=self.logger.logger
        )
    else:
        self._initialize_longitudinal_controller()
    
    self.logger.logger.info(f"Now using {new_type.upper()} controller")

# Usage:
# vehicle.state_handlers['FOLLOWING_LEADER'].switch_controller('cacc')
# vehicle.state_handlers['FOLLOWING_LEADER'].switch_controller('pi', {'kp': 0.2})


# ============================================================
# COMMON SCENARIOS
# ============================================================

# Scenario 1: Highway convoy (fast, consistent speed)
# Use: CACC with moderate spacing
self.controller_type = 'cacc'
params = {'s0': 2.0, 'h': 0.6, 'K': [[0.2, 0.05]]}

# Scenario 2: Urban platooning (stop-and-go)
# Use: CACC with tight spacing, responsive
self.controller_type = 'cacc'
params = {'s0': 1.2, 'h': 0.4, 'K': [[0.25, 0.06]]}

# Scenario 3: Testing/debugging (simple behavior)
# Use: PI for predictable response
self.controller_type = 'pi'
params = {'kp': 0.1, 'ki': 0.8}

# Scenario 4: Mixed conditions (robust)
# Use: Hybrid (CACC when possible, PI as fallback)
self.controller_type = 'hybrid'


# ============================================================
# TROUBLESHOOTING
# ============================================================

# Problem: Vehicle oscillates back and forth
# Solution: Reduce gains
params = {'K': [[0.1, 0.02]]}  # Instead of [[0.2, 0.05]]

# Problem: Vehicle doesn't track spacing well
# Solution: Increase spacing gain
params = {'K': [[0.3, 0.05]]}  # Increase first value

# Problem: Vehicle doesn't match leader speed
# Solution: Increase velocity gain
params = {'K': [[0.2, 0.08]]}  # Increase second value

# Problem: Vehicle too slow to respond
# Solution: Increase acc_to_throttle_gain
params = {'acc_to_throttle_gain': 0.7}  # Instead of 0.5

# Problem: Vehicle too jerky/aggressive
# Solution: Decrease max_throttle and increase filtering
params = {'max_throttle': 0.2, 'alpha_filter': 0.5}


# ============================================================
# PERFORMANCE COMPARISON TEMPLATE
# ============================================================

# Run this to compare controllers:
def compare_controllers():
    """Test all three controllers and log results"""
    
    test_scenarios = [
        ('cacc', {'s0': 1.5, 'h': 0.5, 'K': [[0.2, 0.05]]}),
        ('pi', {'kp': 0.1, 'ki': 1.0}),
        ('hybrid', {})
    ]
    
    for controller_type, params in test_scenarios:
        print(f"\nTesting {controller_type.upper()}...")
        
        # Switch controller
        self.controller_type = controller_type
        self._initialize_longitudinal_controller()
        
        # Run for 10 seconds
        # Log: spacing error, velocity error, throttle command
        
        # Analyze results
        print(f"  Average spacing error: ...")
        print(f"  Average velocity error: ...")
        print(f"  Control effort (throttle variance): ...")
    
    print("\nBest controller for your scenario: ...")


print(__doc__)
