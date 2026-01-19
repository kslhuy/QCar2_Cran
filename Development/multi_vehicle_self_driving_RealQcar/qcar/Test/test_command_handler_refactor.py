
import unittest
import sys
import os
import time
from typing import Dict, Any

# Add parent directory to path to import qcar modules
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from command_handler import CommandHandler
from command_types import CommandType

class MockLogger:
    def __init__(self):
        self.logs = []
        self.logger = self

    def info(self, msg):
        self.logs.append(f"INFO: {msg}")
    
    def log_warning(self, msg, *args):
        self.logs.append(f"WARNING: {msg}")

    def log_error(self, msg, *args):
        self.logs.append(f"ERROR: {msg}")
        
    def warning(self, msg):
        self.logs.append(f"WARNING: {msg}")

class TestCommandHandlerRefactor(unittest.TestCase):
    def setUp(self):
        self.logger = MockLogger()
        self.handler = CommandHandler(self.logger)

    def test_parse_simple_commands(self):
        """Test parsing of simple commands like start, stop"""
        # Test start command
        cmd = {'type': 'start', 'source': 'test'}
        info = self.handler._parse_command(cmd)
        self.assertIsNotNone(info)
        self.assertEqual(info.command_type, CommandType.START)
        
        # Test stop command
        cmd = {'type': 'stop', 'source': 'test'}
        info = self.handler._parse_command(cmd)
        self.assertIsNotNone(info)
        self.assertEqual(info.command_type, CommandType.STOP)

    def test_parse_parameter_commands(self):
        """Test parsing of commands with parameters"""
        # Test set_velocity
        cmd = {'type': 'set_velocity', 'v_ref': 1.5, 'source': 'test'}
        info = self.handler._parse_command(cmd)
        self.assertIsNotNone(info)
        self.assertEqual(info.command_type, CommandType.SET_VELOCITY)
        self.assertEqual(info.data['v_ref'], 1.5)

    def test_platoon_role_handling(self):
        """Test the special logic for enable_platoon role"""
        # Test follower
        cmd = {'type': 'enable_platoon', 'role': 'follower', 'source': 'test'}
        info = self.handler._parse_command(cmd)
        self.assertIsNotNone(info)
        self.assertEqual(info.command_type, CommandType.ENABLE_PLATOON_FOLLOWER)
        
        # Test leader
        cmd = {'type': 'enable_platoon', 'role': 'leader', 'source': 'test'}
        info = self.handler._parse_command(cmd)
        self.assertIsNotNone(info)
        self.assertEqual(info.command_type, CommandType.ENABLE_PLATOON_LEADER)

    def test_scope_commands(self):
        """Test parsing of scope commands"""
        cmd = {'type': 'enable_scope_streaming', 'preset_names': ['test'], 'stream_rate': 10}
        info = self.handler._parse_command(cmd)
        self.assertIsNotNone(info)
        self.assertEqual(info.command_type, CommandType.ENABLE_SCOPE_STREAMING)

    def test_invalid_command(self):
        """Test handling of invalid commands"""
        cmd = {'type': 'non_existent_command', 'source': 'test'}
        info = self.handler._parse_command(cmd)
        self.assertIsNone(info)

if __name__ == '__main__':
    unittest.main()
