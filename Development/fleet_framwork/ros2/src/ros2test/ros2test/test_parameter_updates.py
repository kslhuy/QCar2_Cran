import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import time

class TestParameterUpdates(Node):
    def __init__(self):
        super().__init__('test_parameter_updates')
        
        # Declare test parameters with different types
        self.declare_parameter(
            'test_string', 
            'hello_world', 
            ParameterDescriptor(description='Test string parameter')
        )
        
        self.declare_parameter(
            'test_int', 
            42, 
            ParameterDescriptor(description='Test integer parameter')
        )
        
        self.declare_parameter(
            'test_float', 
            3.14159, 
            ParameterDescriptor(description='Test float parameter')
        )
        
        self.declare_parameter(
            'test_bool', 
            True, 
            ParameterDescriptor(description='Test boolean parameter')
        )
        
        # Store previous values to detect changes
        self.previous_values = {}
        self.change_count = 0
        self.start_time = time.time()
        
        # Create timer to check parameters every 1 second
        self.timer = self.create_timer(1.0, self.check_parameters)
        
        # Initialize previous values
        self.update_previous_values()
        
        self.get_logger().info('=== TEST PARAMETER UPDATES NODE STARTED ===')
        self.get_logger().info('This node monitors parameter changes for testing web interface')
        self.log_current_parameters(initial=True)

    def update_previous_values(self):
        """Update the stored previous values"""
        self.previous_values = {
            'test_string': self.get_parameter('test_string').value,
            'test_int': self.get_parameter('test_int').value,
            'test_float': self.get_parameter('test_float').value,
            'test_bool': self.get_parameter('test_bool').value,
        }

    def check_parameters(self):
        """Check if any parameters have changed and log them"""
        current_time = time.time()
        uptime = current_time - self.start_time
        
        # Get current values
        current_values = {
            'test_string': self.get_parameter('test_string').value,
            'test_int': self.get_parameter('test_int').value,
            'test_float': self.get_parameter('test_float').value,
            'test_bool': self.get_parameter('test_bool').value,
        }
        
        # Check for changes
        changes_detected = []
        for param_name, current_value in current_values.items():
            previous_value = self.previous_values.get(param_name)
            if previous_value != current_value:
                changes_detected.append({
                    'name': param_name,
                    'old': previous_value,
                    'new': current_value,
                    'type': type(current_value).__name__
                })
        
        # Log results
        if changes_detected:
            self.change_count += 1
            self.get_logger().info('🔥 PARAMETER CHANGES DETECTED! 🔥')
            self.get_logger().info(f'Change event #{self.change_count} at uptime {uptime:.1f}s')
            self.get_logger().info('=' * 50)
            
            for change in changes_detected:
                self.get_logger().info(f"📝 {change['name']} [{change['type']}]:")
                self.get_logger().info(f"   OLD: {change['old']}")
                self.get_logger().info(f"   NEW: {change['new']}")
                self.get_logger().info(f"   ✅ CHANGE CONFIRMED!")
            
            self.get_logger().info('=' * 50)
            
            # Update previous values
            self.update_previous_values()
            
        else:
            # No changes - brief status
            self.get_logger().info(f'⏰ Checking parameters... uptime: {uptime:.1f}s, changes detected: {self.change_count}')
        
        # Log current values every 10 seconds regardless
        if int(uptime) % 10 == 0 and uptime > 0:
            self.log_current_parameters()

    def log_current_parameters(self, initial=False):
        """Log all current parameter values"""
        if initial:
            self.get_logger().info('📋 INITIAL PARAMETER VALUES:')
        else:
            self.get_logger().info('📋 CURRENT PARAMETER VALUES:')
        
        self.get_logger().info('-' * 40)
        self.get_logger().info(f"test_string  : '{self.get_parameter('test_string').value}'")
        self.get_logger().info(f"test_int     : {self.get_parameter('test_int').value}")
        self.get_logger().info(f"test_float   : {self.get_parameter('test_float').value}")
        self.get_logger().info(f"test_bool    : {self.get_parameter('test_bool').value}")
        self.get_logger().info('-' * 40)

def main(args=None):
    rclpy.init(args=args)
    node = TestParameterUpdates()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()