# import threading
# from QcarFleet import QcarFleet
# import time

# def test_read(vehicle):
#     for _ in range(10000):
#         try:
#             _, pos, rot, _ = vehicle.get_world_transform()
#             assert isinstance(pos, (list, tuple)), "Position corrupted!"
#         except Exception as e:
#             print("ERROR in read:", e)

# def test_write(vehicle):
#     for _ in range(10000):
#         try:
#             vehicle.set_velocity_and_request_state(forward=0.1, turn=0.0)
#         except Exception as e:
#             print("ERROR in write:", e)

# SimTime = 40
# LeaderIndex = 0
# QcarNum = 5
# DistanceBetweenEachCar          = 0.2
# Controller                      = "CACC"
# Observer                        = ""

# Fleet = QcarFleet(QcarNum, LeaderIndex, DistanceBetweenEachCar, Controller, Observer)
# print("Create Control Threading")
# # LeaderControl = ControlLeader(SimTime, False, [0,1], True)
# print("Leader Control Start")

# v = QcarFleet.Qcars[0]
# t1 = threading.Thread(target=test_read, args=(v,))
# t2 = threading.Thread(target=test_write, args=(v,))
# t1.start()
# t2.start()
# t1.join()
# t2.join()
