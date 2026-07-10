import sys
import os

repo_path = r"c:\Users\Quang Huy Nugyen\Desktop\PHD_paper\Simulation\QCAR\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar"
if repo_path not in sys.path:
    sys.path.insert(0, repo_path)

try:
    from qcar.GUI.qcar_gui.app import create_app
    print("Import successful.")
    app = create_app(num_cars=2)
    print("App created.")
    app.root.update()
    print("GUI Built successfully!")
    app.on_close()
    app.root.destroy()
    print("Cleaned up.")
except Exception as e:
    import traceback
    traceback.print_exc()
    sys.exit(1)
