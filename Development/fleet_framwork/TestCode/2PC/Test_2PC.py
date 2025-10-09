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
    
# Delete any previous QCar instances and stop any running spawn models
qlabs.destroy_all_spawned_actors()
QLabsRealTime().terminate_all_real_time_models()

# Create leader and follower QCars
leader = QLabsQCar2(qlabs)
follower = QLabsQCar2(qlabs)

leader_id = 0
follower_id = 1

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
leader.spawn_id(actorNumber=leader_id, location=[-1.205, -0.83, 0.005], rotation=[0, 0, -44.7],
        scale=[0.1, 0.1, 0.1])
follower.spawn_id(actorNumber=follower_id, location=[-1.735, -0.35, 0.005], 
        rotation=[0, 0, -44.7], scale=[0.1, 0.1, 0.1])

rtModel = os.path.normpath(os.path.join(os.environ['RTMODELS_DIR'], 'QCar2/QCar2_Workspace_studio'))
QLabsRealTime().start_real_time_model(rtModel, actorNumber=0)  

# f = Follower(qcar=follower)
# f.run(leader=leader)

# Cleanup
qlabs.close()
