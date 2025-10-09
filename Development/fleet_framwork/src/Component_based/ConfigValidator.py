"""
Configuration validation for QCar fleet simulation.
Validates vehicle configurations and provides helpful error messages.
"""

import logging
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass


@dataclass
class ValidationError:
    """Represents a configuration validation error."""
    field: str
    message: str
    severity: str = "error"  # error, warning, info


class ConfigValidator:
    """
    Validates vehicle configuration parameters for the QCar fleet simulation.
    """

    def __init__(self, logger: Optional[logging.Logger] = None):
        self.logger = logger or logging.getLogger('config_validator')
        self.errors: List[ValidationError] = []
        self.warnings: List[ValidationError] = []

    def validate_vehicle_config(self, config: Dict[str, Any], vehicle_id: int = None) -> bool:
        """
        Validate a complete vehicle configuration.

        Args:
            config: Vehicle configuration dictionary
            vehicle_id: Vehicle ID for logging (optional)

        Returns:
            True if configuration is valid, False otherwise
        """
        self.errors = []
        self.warnings = []

        vehicle_prefix = f"Vehicle {vehicle_id}: " if vehicle_id is not None else ""

        # Required fields validation
        self._validate_required_fields(config, vehicle_prefix)

        # Type validation
        self._validate_types(config, vehicle_prefix)

        # Value range validation
        self._validate_ranges(config, vehicle_prefix)

        # Logical consistency validation
        self._validate_consistency(config, vehicle_prefix)

        # Network configuration validation
        self._validate_network_config(config, vehicle_prefix)

        # Control configuration validation
        self._validate_control_config(config, vehicle_prefix)

        # GPS configuration validation
        self._validate_gps_config(config, vehicle_prefix)

        # Trust configuration validation
        self._validate_trust_config(config, vehicle_prefix)

        # Log results
        if self.errors:
            for error in self.errors:
                self.logger.error(f"{vehicle_prefix}{error.field}: {error.message}")
            return False

        if self.warnings:
            for warning in self.warnings:
                self.logger.warning(f"{vehicle_prefix}{warning.field}: {warning.message}")

        self.logger.info(f"{vehicle_prefix}Configuration validation passed")
        return True

    def _validate_required_fields(self, config: Dict, prefix: str):
        """Validate that all required fields are present."""
        required_fields = [
            'vehicle_id', 'controller_type', 'is_leader', 'fleet_size',
            'target_ip', 'send_port', 'recv_port', 'ack_port',
            'spawn_location', 'spawn_rotation', 'vehicle_scale'
        ]

        for field in required_fields:
            if field not in config:
                self.errors.append(ValidationError(
                    field=field,
                    message=f"Required field '{field}' is missing"
                ))

        # Conditional required fields
        if config.get('is_leader', False):
            if 'use_physical_qcar' not in config:
                self.warnings.append(ValidationError(
                    field='use_physical_qcar',
                    message="Recommended to specify 'use_physical_qcar' for leader vehicles",
                    severity="warning"
                ))

    def _validate_types(self, config: Dict, prefix: str):
        """Validate field types."""
        type_checks = {
            'vehicle_id': (int, "must be an integer"),
            'fleet_size': (int, "must be an integer"),
            'send_port': (int, "must be an integer"),
            'recv_port': (int, "must be an integer"),
            'ack_port': (int, "must be an integer"),
            'max_steering': (float, "must be a float"),
            'lookahead_distance': (float, "must be a float"),
            'update_rate': (int, "must be an integer"),
            'controller_rate': (int, "must be an integer"),
            'observer_rate': (int, "must be an integer"),
            'gps_update_rate': (int, "must be an integer"),
            'vehicle_scale': ((int, float), "must be a number"),
            'is_leader': (bool, "must be a boolean"),
            'use_physical_qcar': (bool, "must be a boolean"),
            'enable_steering_control': (bool, "must be a boolean"),
            'calibrate': (bool, "must be a boolean"),
            'enable_trust_evaluation': (bool, "must be a boolean"),
            'distance_between_cars': (float, "must be a float"),
            'following_target': ((int, type(None)), "must be an integer or None"),
        }

        for field, (expected_type, message) in type_checks.items():
            if field in config:
                value = config[field]
                if not isinstance(value, expected_type):
                    self.errors.append(ValidationError(
                        field=field,
                        message=message
                    ))

        # Special validations for lists/tuples
        list_fields = {
            'spawn_location': (3, "must be a list/tuple of 3 numbers"),
            'spawn_rotation': (3, "must be a list/tuple of 3 numbers"),
        }

        for field, (expected_length, message) in list_fields.items():
            if field in config:
                value = config[field]
                if not isinstance(value, (list, tuple)):
                    self.errors.append(ValidationError(
                        field=field,
                        message=f"must be a list or tuple"
                    ))
                elif len(value) != expected_length:
                    self.errors.append(ValidationError(
                        field=field,
                        message=f"{message}, got {len(value)} elements"
                    ))
                else:
                    # Check that all elements are numbers
                    for i, elem in enumerate(value):
                        if not isinstance(elem, (int, float)):
                            self.errors.append(ValidationError(
                                field=f"{field}[{i}]",
                                message="must be a number"
                            ))

    def _validate_ranges(self, config: Dict, prefix: str):
        """Validate value ranges."""
        range_checks = {
            'vehicle_id': (0, 100, "must be between 0 and 100"),
            'fleet_size': (1, 20, "must be between 1 and 20"),
            'send_port': (1024, 65535, "must be between 1024 and 65535"),
            'recv_port': (1024, 65535, "must be between 1024 and 65535"),
            'ack_port': (1024, 65535, "must be between 1024 and 65535"),
            'max_steering': (0.1, 1.0, "must be between 0.1 and 1.0"),
            'lookahead_distance': (1.0, 20.0, "must be between 1.0 and 20.0"),
            'update_rate': (10, 200, "must be between 10 and 200 Hz"),
            'controller_rate': (10, 200, "must be between 10 and 200 Hz"),
            'observer_rate': (10, 200, "must be between 10 and 200 Hz"),
            'gps_update_rate': (1, 50, "must be between 1 and 50 Hz"),
            'vehicle_scale': (0.1, 5.0, "must be between 0.1 and 5.0"),
            'distance_between_cars': (2.0, 20.0, "must be between 2.0 and 20.0 meters"),
        }

        for field, (min_val, max_val, message) in range_checks.items():
            if field in config:
                value = config[field]
                if isinstance(value, (int, float)):
                    if not (min_val <= value <= max_val):
                        self.errors.append(ValidationError(
                            field=field,
                            message=message
                        ))

    def _validate_consistency(self, config: Dict, prefix: str):
        """Validate logical consistency between fields."""
        # Port uniqueness
        ports = ['send_port', 'recv_port', 'ack_port']
        port_values = []
        for port_field in ports:
            if port_field in config:
                port_val = config[port_field]
                if port_val in port_values:
                    self.errors.append(ValidationError(
                        field=port_field,
                        message=f"Port {port_val} is used by multiple fields - ports must be unique"
                    ))
                port_values.append(port_val)

        # Leader/follower consistency
        is_leader = config.get('is_leader', False)
        following_target = config.get('following_target')

        if is_leader and following_target is not None:
            self.errors.append(ValidationError(
                field='following_target',
                message="Leader vehicles should not have a following_target"
            ))

        if not is_leader and following_target is None:
            self.warnings.append(ValidationError(
                field='following_target',
                message="Follower vehicles should specify a following_target",
                severity="warning"
            ))

        # Physical QCar consistency
        use_physical_qcar = config.get('use_physical_qcar', False)
        if use_physical_qcar and not is_leader:
            self.warnings.append(ValidationError(
                field='use_physical_qcar',
                message="Physical QCar is typically only used for leader vehicles",
                severity="warning"
            ))

        # Control observer mode consistency
        use_control_observer_mode = config.get('use_control_observer_mode', False)
        if use_control_observer_mode and not use_physical_qcar:
            self.warnings.append(ValidationError(
                field='use_control_observer_mode',
                message="Control observer mode is only meaningful with physical QCar",
                severity="warning"
            ))

    def _validate_network_config(self, config: Dict, prefix: str):
        """Validate network-related configuration."""
        # IP address validation
        target_ip = config.get('target_ip')
        if target_ip:
            if not self._is_valid_ip(target_ip):
                self.errors.append(ValidationError(
                    field='target_ip',
                    message=f"'{target_ip}' is not a valid IP address"
                ))

        # Peer ports validation
        peer_ports = config.get('peer_ports', {})
        if isinstance(peer_ports, dict):
            for vehicle_id, ports in peer_ports.items():
                if not isinstance(vehicle_id, int):
                    self.errors.append(ValidationError(
                        field=f'peer_ports[{vehicle_id}]',
                        message="Vehicle ID must be an integer"
                    ))
                elif isinstance(ports, dict):
                    required_peer_ports = ['send', 'recv', 'ack']
                    for port_type in required_peer_ports:
                        if port_type not in ports:
                            self.errors.append(ValidationError(
                                field=f'peer_ports[{vehicle_id}][{port_type}]',
                                message=f"Missing required port type '{port_type}'"
                            ))
                        elif not isinstance(ports[port_type], int):
                            self.errors.append(ValidationError(
                                field=f'peer_ports[{vehicle_id}][{port_type}]',
                                message="Port must be an integer"
                            ))
                        elif not (1024 <= ports[port_type] <= 65535):
                            self.errors.append(ValidationError(
                                field=f'peer_ports[{vehicle_id}][{port_type}]',
                                message="Port must be between 1024 and 65535"
                            ))

        # Communication mode validation
        comm_mode = config.get('communication_mode', 'unidirectional')
        valid_modes = ['unidirectional', 'bidirectional']
        if comm_mode not in valid_modes:
            self.errors.append(ValidationError(
                field='communication_mode',
                message=f"Must be one of {valid_modes}, got '{comm_mode}'"
            ))

    def _validate_control_config(self, config: Dict, prefix: str):
        """Validate control-related configuration."""
        controller_type = config.get('controller_type')
        valid_controllers = ['CACC', 'IDM', 'Dummy']

        if controller_type and controller_type not in valid_controllers:
            self.errors.append(ValidationError(
                field='controller_type',
                message=f"Must be one of {valid_controllers}, got '{controller_type}'"
            ))

        # Controller-specific validations
        if controller_type == 'CACC':
            # CACC might need additional parameters
            pass
        elif controller_type == 'IDM':
            # IDM might need additional parameters
            pass

    def _validate_gps_config(self, config: Dict, prefix: str):
        """Validate GPS-related configuration."""
        gps_server_ip = config.get('gps_server_ip')
        if gps_server_ip and not self._is_valid_ip(gps_server_ip):
            self.errors.append(ValidationError(
                field='gps_server_ip',
                message=f"'{gps_server_ip}' is not a valid IP address"
            ))

        gps_server_port = config.get('gps_server_port')
        if gps_server_port is not None:
            if not isinstance(gps_server_port, int):
                self.errors.append(ValidationError(
                    field='gps_server_port',
                    message="Must be an integer"
                ))
            elif not (1024 <= gps_server_port <= 65535):
                self.errors.append(ValidationError(
                    field='gps_server_port',
                    message="Must be between 1024 and 65535"
                ))

    def _validate_trust_config(self, config: Dict, prefix: str):
        """Validate trust-related configuration."""
        enable_trust = config.get('enable_trust_evaluation', False)
        connected_vehicles = config.get('connected_vehicles', [])

        if enable_trust:
            if not isinstance(connected_vehicles, list):
                self.errors.append(ValidationError(
                    field='connected_vehicles',
                    message="Must be a list when trust evaluation is enabled"
                ))
            else:
                for i, veh_id in enumerate(connected_vehicles):
                    if not isinstance(veh_id, int):
                        self.errors.append(ValidationError(
                            field=f'connected_vehicles[{i}]',
                            message="Must be an integer"
                        ))

            # Fleet graph validation
            fleet_graph = config.get('fleet_graph', [])
            if fleet_graph:
                if not isinstance(fleet_graph, list):
                    self.errors.append(ValidationError(
                        field='fleet_graph',
                        message="Must be a list"
                    ))
                elif len(fleet_graph) != config.get('fleet_size', 0):
                    self.errors.append(ValidationError(
                        field='fleet_graph',
                        message=f"Must have {config.get('fleet_size', 0)} rows for fleet_size"
                    ))

    def _is_valid_ip(self, ip: str) -> bool:
        """Check if string is a valid IP address."""
        import ipaddress
        try:
            ipaddress.ip_address(ip)
            return True
        except ValueError:
            return False

    def get_validation_summary(self) -> Dict[str, Any]:
        """Get a summary of validation results."""
        return {
            'valid': len(self.errors) == 0,
            'errors': [{'field': e.field, 'message': e.message} for e in self.errors],
            'warnings': [{'field': w.field, 'message': w.message} for w in self.warnings],
            'error_count': len(self.errors),
            'warning_count': len(self.warnings)
        }


def validate_fleet_config(fleet_config: List[Dict[str, Any]], logger: Optional[logging.Logger] = None) -> bool:
    """
    Validate an entire fleet configuration.

    Args:
        fleet_config: List of vehicle configurations
        logger: Logger instance

    Returns:
        True if all configurations are valid
    """
    validator = ConfigValidator(logger)
    all_valid = True

    for i, vehicle_config in enumerate(fleet_config):
        vehicle_id = vehicle_config.get('vehicle_id', i)
        if not validator.validate_vehicle_config(vehicle_config, vehicle_id):
            all_valid = False

    return all_valid