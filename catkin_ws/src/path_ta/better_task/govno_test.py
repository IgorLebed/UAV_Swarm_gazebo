import time


reach_position = []
goal_pose_x = None
goal_pose_y = None
q_number = 0
check = True
while (check == True):
    try:
        if (goal_pose_x == None and goal_pose_y == None):
            for i in range(0, 2):
                target_position_x = 1
                target_position_y = 1
                print("Taget Pos: %s and %s",target_position_x, target_position_y)
                time.sleep(5)
            goal_pose_x = 2
            goal_pose_y = 2
        else:
        
            if (q_number < 2):
                print("This is number!")
                print("Goal pose: x=%s and y=%s", str(goal_pose_x), str(goal_pose_y))  
                #position = (int(goal_pose_x), int(goal_pose_y), 10)
                takeoff_height = 15
                #reach_position(int(goal_pose_x), int(goal_pose_y), takeoff_height, 30)  
                #print("reach position: %s", reach_position) 
                time.sleep(3)
                q_number += 1 
            else:      
                check = False
                        
    except ValueError:
        print("Error")
        check = False
