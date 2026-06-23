# test manually or as a script
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core.vehicle_state_machine import State, StateMachine
from core.types import GuiCommand

sm = StateMachine()
assert sm.state == State.INITIALIZING
assert not sm.should_drive()

sm.mark_ready()
assert sm.state == State.READY
assert not sm.should_drive()

sm.handle_command(GuiCommand(command="START", payload={}))
assert sm.state == State.RUNNING
assert sm.should_drive()

sm.handle_command(GuiCommand(command="EMERGENCY_STOP", payload={}))
assert sm.state == State.STOPPED
assert not sm.should_drive()

sm.handle_command(GuiCommand(command="RESET", payload={}))
assert sm.state == State.READY

print("All assertions passed!")