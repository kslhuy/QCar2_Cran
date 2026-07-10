# QCar Fleet Controller — GUI 📡

**Lightweight Tkinter ground-station for monitoring and controlling a fleet of QCar vehicles.**

---

## 🚀 Overview
A compact GUI for live telemetry, manual control, platoon and V2V management, remote scope streaming, and command logging for QCar fleets.

## ✨ Key Features
- Live telemetry panels (position, velocity, heading, state) for multiple vehicles
- Per-vehicle manual control (keyboard / remote input)
- Platoon setup and trigger (assign positions, choose leader)
- V2V activation and network monitoring
- Perception control (enable/disable YOLO on vehicles)
- Remote scope streaming and viewer integration
- Command logging and fleet statistics (commands sent/failed, uptime, success rate)

## 🏗️ Architecture (brief)
- `app.py` — **QCarFleetController**: main app, GUI layout, update loop, and high-level command orchestration
- `controllers/` — network and input logic (`QCarRemoteController`, `ManualInputController`)
- `widgets/` — UI components (car panels, fleet controls, status/log panels)
- `config.py` — application/network/GUI configuration dataclasses
- `theme.py` — theming and style helpers

The GUI runs a small background thread to poll telemetry and uses the `LogPanel` and `StatusPanel` widgets to surface info.

## ▶️ Quick Start
**Requirements:** Python 3.x and Tkinter available on your system.

Run from the repository root:
```bash
python -m qcar_gui.app
```
Or create programmatically:
```python
from qcar_gui.app import create_app
app = create_app(num_cars=5, host_ip='0.0.0.0', base_port=5000)
app.root.mainloop()
```

Default network: `host_ip='0.0.0.0'`, `base_port=5000`. Configure values in `AppConfig` or pass to `create_app()`.

## ⚙️ Usage Notes
- Ensure the chosen ports are available and firewall rules allow connections from vehicles.
- Toggle manual control per vehicle from each `CarPanel`. The `ManualInputController` handles keyboard/remote input.
- Use `Fleet Controls` to setup/trigger platoons, activate V2V, and control perception for all vehicles.

## 🛠️ Development
- Add commands by extending `QCarRemoteController` and handling them in `app.py`.
- Add new UIs by creating widgets under `widgets/` and integrating them into `_build_gui()`.
- Keep UI updates on the main thread; use `root.after(...)` for scheduling UI changes from worker threads.

## ⚠️ Troubleshooting
- If vehicles cannot connect: check host IP/port, firewall, and that `QCarRemoteController` is running.
- If the GUI freezes: ensure long-running tasks do not run on Tkinter's main thread.
- Check the in-GUI **Log Panel** for detailed errors and status messages.

---

If you want, I can add a short `USAGE.md` with screenshots or include a small mock vehicle for local testing.