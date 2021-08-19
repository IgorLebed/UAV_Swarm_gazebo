from sys import path
import setpoint_listener as path_ta
import time

while True:
    path_ta.pos_listener()
    x = path_ta.goal_pose_x
    y = path_ta.goal_pose_y

    print("This is x= %s, y= %s", str(x), str(y))
    time.sleep(3)