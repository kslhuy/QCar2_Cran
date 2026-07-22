# test manually or as a script
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.vehicle_state_machine import State, StateMachine
from core.commands import CommandType, VehicleCommand

sm = StateMachine()
assert sm.state == State.INITIALIZING
assert not sm.should_drive()

sm.mark_ready()
assert sm.state == State.READY
assert not sm.should_drive()

sm.handle_command(VehicleCommand(command_type=CommandType.START))
assert sm.state == State.RUNNING
assert sm.should_drive()

sm.handle_command(VehicleCommand(command_type=CommandType.EMERGENCY_STOP))
assert sm.state == State.STOPPED
assert not sm.should_drive()

sm.handle_command(VehicleCommand(command_type=CommandType.RESET))
assert sm.state == State.READY

print("All assertions passed!")
