"""
Performs task allocation given number of drones online
"""
import rospy
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import Pose, PoseStamped
import random
import numpy as np
import os
import json


def generate_tasks(num_task, min_pos, max_pos, height):
    task_list = []
    for i in range(num_task):
        task = (random.uniform(min_pos, max_pos), random.uniform(min_pos, max_pos), height)
        task_list.append(task)
    return task_list

def generate_json(height):
    json_data = []
    task_list = []
    x_arr = []
    y_arr = []
    if os.path.exists('/home/igor/UAV_Swarm_gazebo/catkin_ws/src/path_ta/json/Targets.json'):
        with open('/home/igor/UAV_Swarm_gazebo/catkin_ws/src/path_ta/json/Targets.json', 'r') as d: 
            json_data = json.load(d) 
            target_point_arr = json_data.get('points')
        for i in range(len(target_point_arr)):
            target_point_arr_ = target_point_arr[i]
            x = target_point_arr_[0]
            y = target_point_arr_[1] 
            x_arr.append(x)
            y_arr.append(y)
            centroid = [sum(x_arr) / len(x_arr), sum(y_arr) / len(y_arr)]
            task = (x, y, height)
            task_list.append(task)
        return task_list
    else:
        print('####!!!!There is no file with targets!!!!####')

# task_list = [
# (-957.3915945077315, -855.0138749238104, 60),
# (-910.8959478922188, -895.5183857586235, 50),
# (-948.4347058273852, -921.3926183544099, 60),
# (-875.1556231221184, -947.2644253643230, 50),
# (-845.2041011163965, -858.3795844987035, 60),
# (-858.6145041380078, -861.4421726837754, 50),
# (-919.3042095946148, -895.0775583777577, 60),
# (-942.3035844964907, -844.3298955513164, 50),
# (-933.9325257679448, -922.8892891705036, 60),
# (-889.9556830069050, -887.7070486117154, 50)] 

num_task = 4
num_robots = 2
max_pose = 200
min_pos = -200

# task_list = generate_tasks(num_task, min_pos, max_pose, 20)  # randomly generate tasks
task_list = generate_json(60)
# task_list = [(-9, -9, 20), (-9, 9, 20), (9, -9, 20), (9, 9, 20), (-4, -4, 20), (4, -4, 20), (-4, 4, 20), (4, 4, 20),
#              (0, 10, 20), (10, 0, 20), (0, -10, 20), (-10, 0, 20)]
# task_list = [
#  (-910, -895, 50), (-870, -950, 50), (-900, -880, 50), (-942, -830, 50), (-850, -890, 50), (-900, -830, 50)]
# using 1 and 0 to represent true or false
task_status = np.zeros(len(task_list))  # recording finished tasks
robot_curr_task_id = np.empty(num_robots)  # records which task is currently working on
rem_task_list = np.empty(len(task_list))
first_task = np.ones(num_robots)
task_done = np.zeros(num_robots)
mission_done = np.zeros(num_robots)
task_count = 0

robot_pos = np.empty((num_robots, 3))

'publisher for tasks'
curr_task_pub0 = rospy.Publisher('/bomber1/curr_task', PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub1 = rospy.Publisher('/bomber2/curr_task', PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub2 = rospy.Publisher('/bomber3/curr_task', PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub = [curr_task_pub0, curr_task_pub1, curr_task_pub2]

'publisher for mission completion'
mission_complete_pub = rospy.Publisher('/mission_bool', Bool, queue_size=1)


def assign_tasks(_n_robot, r_pos, _tasks):
    task_id = np.empty(len(_tasks))  # which robot gets the task
    for t in range(len(_tasks)):
        cost_list = np.zeros(_n_robot)
        for n in range(_n_robot):
            tx, ty, tz = _tasks[t]
            rx, ry, rz = r_pos[n]
            # rospy.loginfo(rx)
            cost = (tx - rx) * (tx - rx) + (ty - ry) * (ty - ry)
            cost_list[n] = cost
        task_id[t] = np.argmin(cost_list)
    robot_tasks = []
    for n in range(_n_robot):
        robot_tasks.append([])
    for t in range(len(task_id)):
        r_id = task_id[t]
        robot_tasks[int(r_id)].append(_tasks[t])
    return task_id, robot_tasks


def online_task_assign(r_pos, _tasks, _task_status):
    rx, ry, rz = r_pos
    cost_list = np.empty(len(_tasks))
    _count = 0
    for i, t in enumerate(_tasks):
        if _task_status[i] == 0:  # task is available
            tx, ty, tz = t
            cost_list[i] = (tx - rx) * (tx - rx) + (ty - ry) * (ty - ry)
        else:  # 1 or 2
            cost_list[i] = 2 * (max_pose - min_pos) * (max_pose - min_pos)  # max possible cost to prevent assignment
            _count += 1
    if _count == num_task:
        _no_task = True
    elif _count >= num_task:
        _no_task = True
        rospy.loginfo("WARNING: COUNT IS BIGGER THAN TASK COUNT!")
    else:
        _no_task = False
    task_id = np.argmin(cost_list)
    next_task = _tasks[task_id]
    return int(task_id), next_task, _no_task


def status_callback0(data):
    # rospy.loginfo('robot 0 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 0 idling")
        global task_done
        task_done[0] = 1


def status_callback1(data):
    # rospy.loginfo('robot 1 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 1 idling")
        global task_done
        task_done[1] = 1


def status_callback2(data):
    # rospy.loginfo('robot 2 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 2 idling")
        global task_done
        task_done[2] = 1


def robot0_pos_callback(msg):
    robot_pos[0][0] = msg.pose.position.x
    robot_pos[0][1] = msg.pose.position.y
    robot_pos[0][2] = msg.pose.position.z


def robot1_pos_callback(msg):
    robot_pos[1][0] = msg.pose.position.x
    robot_pos[1][1] = msg.pose.position.y
    robot_pos[1][2] = msg.pose.position.z


def robot2_pos_callback(msg):
    robot_pos[2][0] = msg.pose.position.x
    robot_pos[2][1] = msg.pose.position.y
    robot_pos[2][2] = msg.pose.position.z


def publish_task_position(curr_task_pub0, pos):
    waypoint = PoseStamped()
    waypoint.header.stamp = rospy.Time.now()
    waypoint.pose.position.x = pos[0]
    waypoint.pose.position.y = pos[1]
    waypoint.pose.position.z = pos[2]
    curr_task_pub0.publish(waypoint)
    rospy.loginfo('current task position {}'.format(pos))


if __name__ == '__main__':
    try:
        rospy.init_node('master_online_task_alloc_node', anonymous=True)

        # TODO: wait for service from robots to confirm they are ready for tasks

        'subscriber for task completion'
        robot_status_sub0 = rospy.Subscriber('/bomber1/task_bool', Bool, status_callback0)  # sub to robot's done with task
        robot_status_sub1 = rospy.Subscriber('/bomber2/task_bool', Bool, status_callback1)
        robot_status_sub2 = rospy.Subscriber('/bomber3/task_bool', Bool, status_callback2)

        'subscriber for robots position'
        robot_pos_sub0 = rospy.Subscriber('/bomber1/mavros/local_position/pose', PoseStamped, robot0_pos_callback)
        robot_pos_sub1 = rospy.Subscriber('/bomber2/mavros/local_position/pose', PoseStamped, robot1_pos_callback)
        robot_pos_sub2 = rospy.Subscriber('/bomber3/mavros/local_position/pose', PoseStamped, robot2_pos_callback)

        rate = rospy.Rate(3)
        rospy.sleep(3)
        rospy.loginfo("planning now, robot pose:"+str(robot_pos))
        rospy.sleep(1)
        # initially assign tasks to each robot
        for n in range(num_robots):
            curr_task_id, curr_task, no_task = online_task_assign(robot_pos[n], task_list, task_status)
            robot_curr_task_id[n] = int(curr_task_id)
            task_status[curr_task_id] = 1
            publish_task_position(curr_task_pub[n], curr_task)
            task_done[n] = 0
            task_count += 1
        rospy.loginfo("task assignments finished! starting mission")

        while not rospy.is_shutdown():
            for i in range(num_robots):  # iterate through robots
                if task_done[i] == 1:  # if robot is done with current task
                    task_status[int(robot_curr_task_id[i])] = 2
                    curr_task_id, curr_task, no_task = online_task_assign(robot_pos[i], task_list, task_status)
                    if no_task:
                        rospy.loginfo("no more task! waiting for mission to finish")
                    else:
                        robot_curr_task_id[i] = curr_task_id
                        task_status[int(robot_curr_task_id[i])] = 1
                        publish_task_position(curr_task_pub[i], curr_task)
                        task_done[i] = 0
                        task_count += 1

            # termination check
            if np.sum(task_status) == num_task * 2:
                tf = 10
                while tf > 0:  # wait to shutdown node
                    mission_complete_pub.publish(True)
                    rospy.loginfo("Mission accomplished! Terminating the node in "+str(tf))
                    tf -= 1
                    rospy.sleep(1)
                rospy.loginfo("Number of tasks completed: "+str(task_count))
                break

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
