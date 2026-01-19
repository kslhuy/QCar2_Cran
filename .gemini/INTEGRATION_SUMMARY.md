# QCar Ground Station Integration Summary

## Problem Fixed

The Web Ground Station was not receiving proper vehicle status messages from the Python Ground Station bridge, preventing dynamic vehicle panel creation and telemetry updates.

## Changes Made

### 1. Fixed Python GS Bridge (`remote_controller.py`)

#### **Issue 1: Missing `Set` import**
- **Problem**: Type annotation for `websocket_clients` was using `Set` but it wasn't imported
- **Fix**: Added `Set` to imports from `typing`

#### **Issue 2: No vehicle connection broadcasts**
- **Problem**: When vehicles connected to Python GS, WebSocket clients weren't notified
- **Fix**: Added `_broadcast_to_websockets()` call in `_accept_connection()` to send `vehicle_status` message with type "connected"

#### **Issue 3: No vehicle disconnection broadcasts**  
- **Problem**: When vehicles disconnected, WebSocket clients weren't notified
- **Fix**: Added `_broadcast_to_websockets()` call in `_cleanup_car()` to send `vehicle_status` message with type "disconnected"

#### **Issue 4: Initial status not sent to new WebSocket connections**
- **Already Fixed**: The `_handle_websocket_client()` function properly sends all currently connected vehicles' status when a new WebSocket client connects

## Data Flow (Fixed)

### 1. Web GS Loads → Auto-connects to ws://localhost:8080 ✅
**Implementation**: `App.tsx` (lines 67-83)
```typescript
useEffect(() => {
  bridgeService.connect(); // Auto-connect on mount
  ...
}, []);
```

### 2. Python GS accepts connection → Sends initial vehicle_status for all connected cars ✅
**Implementation**: `remote_controller.py` (lines 281-290)
```python
async def _handle_websocket_client(self, websocket: WebSocketServerProtocol):
    # Send initial status for all connected cars
    for car_id, car in self.cars.items():
        if car.status == 'connected':
            await websocket.send(json.dumps({
                "type": "vehicle_status",
                "vehicle_id": f"qcar-{car_id}",
                "status": "connected",
                ...
            }))
```

### 3. Web GS receives vehicle_status → Creates vehicle panels dynamically ✅
**Implementation**: `App.tsx` (lines 165-210)
```typescript
const unsubVehicleStatus = bridgeService.onVehicleStatus((msg: VehicleStatusMessage) => {
  if (msg.status === 'connected') {
    // Add new vehicle or update existing
  } else {
    // Mark as disconnected
  }
});
```

### 4. Python GS broadcasts telemetry → Web GS updates positions on map ✅
**Implementation**: 
- **Python side**: `remote_controller.py` (lines 507-511) - broadcasts all telemetry to WebSockets
- **Web side**: `App.tsx` (lines 86-162) - merges telemetry data into vehicle state

### 5. Web GS sends commands → Python GS forwards to vehicles ✅
**Implementation**:
- **Web side**: `websocketBridgeService.ts` - sends commands with `target` field
- **Python side**: `remote_controller.py` (lines 329-343) - parses `target` and forwards to specific vehicle or all

## Message Types

### From Python GS to Web GS:

1. **vehicle_status** - Connection/disconnection events
```json
{
  "type": "vehicle_status",
  "vehicle_id": "qcar-0",
  "status": "connected" | "disconnected",
  "ip": "192.168.x.x",
  "port": 5000
}
```

2. **telemetry** - High-frequency vehicle data (~10Hz)
```json
{
  "type": "telemetry",
  "vehicle_id": "qcar-0",
  "x": 0.0, "y": 0.0, "theta": 0.0,
  "v": 1.0, "battery": 100,
  "state": "FOLLOWING_PATH",
  ...
}
```

3. **v2v_status** - Low-frequency periodic status (~1Hz)
```json
{
  "type": "v2v_status",
  "vehicle_id": "qcar-0",
  "v2v_active": true,
  "v2v_peers": 2,
  "platoon_enabled": true,
  "local_observer_type": "ekf",
  "longitudinal_ctrl_type": "cacc",
  ...
}
```

### From Web GS to Python GS:

Commands include `target` field to route to specific vehicle or "all":

```json
{
  "type": "start",
  "target": "qcar-0" | "all"
}
```

## Testing Recommendations

1. **Start Python GS first**:
   ```bash
   cd Development/multi_vehicle_self_driving_RealQcar
   python -m qcar.GUI.qcar_gui.main
   ```

2. **Start Web GS**:
   ```bash
   cd GroundStation-Qcar-App
   npm run dev
   ```

3. **Connect vehicles to Python GS** (port 5000, 5001, etc.)

4. **Open Web GS** in browser at `http://localhost:3000` or similar

5. **Verify**:
   - Web GS auto-connects to WebSocket (Bridge status shows CONNECTED)
   - Vehicle panels appear dynamically when vehicles connect to Python GS
   - Telemetry updates in real-time (position, velocity, battery, state)
   - Commands from Web GS work (Start, Stop, E-Stop)
   - Vehicle disconnection is reflected in Web GS

## Known Vehicle ID Format

- **Python internal**: Integer `car_id` (0, 1, 2, ...)
- **WebSocket messages**: String `vehicle_id` as "qcar-0", "qcar-1", etc.
- **Web GS internal**: Uses WebSocket format "qcar-0"

The conversion happens in `remote_controller.py` when broadcasting:
```python
ws_data['vehicle_id'] = f"qcar-{car_id}"
```

## Additional Notes

- The Python GS still maintains its own Tkinter GUI independently
- The Web GS is a read-only visualization layer but can send commands
- Both GUIs can operate simultaneously
- WebSocket reconnection is handled automatically (max 10 attempts, 3s interval)
