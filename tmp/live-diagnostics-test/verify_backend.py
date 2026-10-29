"""Exercise actual payload builders without importing hardware dependencies."""
import ast
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]


def load_method(path, name, namespace):
    tree = ast.parse((ROOT / path).read_text(encoding="utf-8"))
    method = next(node for node in ast.walk(tree)
                  if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
    return namespace[name]


build = load_method(
    "Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_logic.py",
    "_build_telemetry_data", {"time": SimpleNamespace(time=lambda: 1000.0)},
)
vehicle = SimpleNamespace(
    _telemetry_state={}, elapsed_time=lambda: 5.0,
    _get_cached_control_state=lambda: dict(x=1, y=2, theta=0, velocity=0.5, gps_valid=True),
    state_machine=SimpleNamespace(state=SimpleNamespace(name="IDLE")), opponent_data=[],
)
assert build(vehicle)["loop_execution_ms"] is None  # No completed control iteration yet.
vehicle._last_loop_execution_ms = 2.75
payload = build(vehicle)
assert payload["loop_execution_ms"] == 2.75
assert payload["v"] == 0.5 and payload["state"] == "IDLE"
assert json.loads(json.dumps(payload))["loop_execution_ms"] == 2.75

handle = load_method(
    "Development/multi_vehicle_self_driving_RealQcar/qcar/GUI/qcar_gui/controllers/remote_controller.py",
    "_handle_websocket_message", {"json": json},
)


async def verify_ping():
    replies = []

    async def send(payload):
        replies.append(json.loads(payload))

    socket = SimpleNamespace(send=send)
    await handle(SimpleNamespace(), socket, json.dumps({"type": "ping", "ping_id": 42}))
    assert replies.pop() == {"type": "pong", "ping_id": 42}
    await handle(SimpleNamespace(), socket, json.dumps({"type": "ping"}))
    assert replies.pop()["type"] == "pong"  # Existing browser clients still work.


asyncio.run(verify_ping())
print("PASS: actual vehicle timing payload and bridge ping/pong, including legacy requests")
