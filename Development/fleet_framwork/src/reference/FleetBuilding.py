from follower_multicar import Follower
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
import time, os, math, threading
import numpy as np

from qvl.basic_shape import QLabsBasicShape
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk

from Controller.idm_control import IDMControl
from Controller.CACC import CACC

USE_CACC = True 

class DummyController:
    def __init__(self,qcar_id):
        self.param_opt = {
            'alpha': 1.0,
            'beta': 1.5,
            'v0': 1.0,
            'delta': 4,
            'T': 0.4,
            's0': 1,  # for IDM
            'ri': 0.5,  # for CACC
            'hi': 0.5,
            'K': np.array([[1, 0.0], [0.0, 1]])  # for CACC
        }
        self.param_sys = None
        self.goal = None
        self.straightlane = None
        self.vehicle_id = qcar_id

    def get_surrounding_vehicles(self, *args, **kwargs):
        return None, [dummy_leader], None, None  # You override this anyway


# Connect to QLabs
os.system('cls')
qlabs = QuanserInteractiveLabs()
print("Connecting to QLabs...")
try:
    qlabs.open("localhost")
    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()
    print("Connected to QLabs")
except:
    print("Unable to connect to QLabs")
    quit()
    
# Delete any previous QCar instances and stop any running spawn models


# Create leader and follower QCars
leader    = QLabsQCar2(qlabs)
follower1 = QLabsQCar2(qlabs)
follower2 = QLabsQCar2(qlabs)
follower3 = QLabsQCar2(qlabs)
follower4 = QLabsQCar2(qlabs)

leader_id = 0
follower1_id = 1
follower2_id = 2
follower3_id = 3
follower4_id = 4

x_offset = 0.13
y_offset = 1.67
hFloor = QLabsQCarFlooring(qlabs)
hFloor.spawn_degrees([x_offset, y_offset, 0.001],rotation = [0, 0, -90], configuration=0)


### region: Walls
hWall = QLabsWalls(qlabs)
hWall.set_enable_dynamics(False)

for y in range (5):
    hWall.spawn_degrees(location=[-2.4 + x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])

for x in range (5):
    hWall.spawn_degrees(location=[-1.9+x + x_offset, 3.05+ y_offset, 0.001], rotation=[0, 0, 90])

for y in range (6):
    hWall.spawn_degrees(location=[2.4+ x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])

for x in range (4):
    hWall.spawn_degrees(location=[-0.9+x+ x_offset, -3.05+ y_offset, 0.001], rotation=[0, 0, 90])

hWall.spawn_degrees(location=[-2.03 + x_offset, -2.275+ y_offset, 0.001], rotation=[0, 0, 48])
hWall.spawn_degrees(location=[-1.575+ x_offset, -2.7+ y_offset, 0.001], rotation=[0, 0, 48])


# Spawning crosswalks
myCrossWalk = QLabsCrosswalk(qlabs)
myCrossWalk.spawn_degrees   (location =[-2 + x_offset, -1.475 + y_offset, 0.01],
                            rotation=[0,0,0], 
                            scale = [0.1,0.1,0.075],
                            configuration = 0)

mySpline = QLabsBasicShape(qlabs)
mySpline.spawn_degrees (location=[2.05 + x_offset, -1.5 + y_offset, 0.01], 
                        rotation=[0, 0, 0], 
                        scale=[0.27, 0.02, 0.001], 
                        waitForConfirmation=False)

# Spawn cars
leader.spawn_id(actorNumber=leader_id, location=[-1.205, -0.83, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])
follower1.spawn_id(actorNumber=follower1_id, location=[-1.735, -0.35, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])
follower2.spawn_id(actorNumber=follower2_id, location=[-2, 0.63, 0.005], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])
follower3.spawn_id(actorNumber=follower3_id, location=[-1.905, 0.914, 0.006], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])
follower4.spawn_id(actorNumber=follower4_id, location=[-1.964, 2.273, 0.006], rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])

rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
QLabsRealTime().start_real_time_model(rtModel, actorNumber=leader_id)

## Choose Controller ##
controller_follower1 = DummyController(follower1_id)
controller_follower2 = DummyController(follower2_id)
controller_follower3 = DummyController(follower3_id)
controller_follower4 = DummyController(follower4_id)
if USE_CACC:
    control_algo1 = CACC(controller_follower1)
    control_algo2 = CACC(controller_follower2)
    control_algo3 = CACC(controller_follower3)
    control_algo4 = CACC(controller_follower4)
else:
    control_algo1 = IDMControl(controller_follower1)
    control_algo2 = IDMControl(controller_follower2)

TaskFollower1 = Follower(qcar=follower1, idm_controller=control_algo1, vehicle_id=follower1_id)
TaskFollower2 = Follower(qcar=follower2, idm_controller=control_algo2, vehicle_id=follower2_id)
TaskFollower3 = Follower(qcar=follower3, idm_controller=control_algo3, vehicle_id=follower3_id)
TaskFollower4 = Follower(qcar=follower4, idm_controller=control_algo4, vehicle_id=follower4_id)

TaskFollower1.running = True
TaskFollower2.running = True

print("Using CACC" if USE_CACC else "Using IDM")

counter = 0
t=time.time()
while TaskFollower1.running and TaskFollower2.running == True:
    ts = t
    t=time.time()
    dt = t-ts
    if counter%10==0:
        print(dt)
    TaskFollower1.follow(leader)
    TaskFollower2.follow(follower1)
    TaskFollower3.follow(follower2)
    TaskFollower4.follow(follower3)
    time.sleep(0.01)
    counter += 1