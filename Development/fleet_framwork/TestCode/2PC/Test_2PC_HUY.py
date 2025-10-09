from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
import time, os, math

from qvl.basic_shape import QLabsBasicShape
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk

# Connect to QLabs
os.system('cls')
qlabs = QuanserInteractiveLabs()
print("Connecting to QLabs...")
try:
    qlabs.open("169.254.88.176")
    print("Connected to QLabs")
except:
    print("Unable to connect to QLabs")
    quit()

rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))

# # Delete any previous QCar instances and stop any running spawn models
# qlabs.destroy_all_spawned_actors()
# QLabsRealTime().terminate_real_time_model(rtModel)
QLabsRealTime().terminate_all_real_time_models()


# Create leader and follower QCars
Huy_car = QLabsQCar2(qlabs)
Huy_car_id = 2

# Spawn cars
# Huy_car.spawn_id(actorNumber=Huy_car_id, location=[-1.735, -0.35, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])

QLabsRealTime().start_real_time_model(rtModel, actorNumber=Huy_car_id , QLabsHostName="169.254.88.176")  

# f = Follower(qcar=follower)
# f.run(leader=leader)

# Cleanup
qlabs.close()
