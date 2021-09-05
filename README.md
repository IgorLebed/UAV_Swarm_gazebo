# UAV_Swarm_gazebo

## Introduction

## Preparation
```
cd $HOME
git clone https://github.com/IgorLebed/UAV_Swarm_gazebo.git --recursive
```
```
cd UAV_Swarm_gazebo/Autopilot/
bash ./Tools/setup/ubuntu.sh
```

```
cd ~/UAV_Swarm_gazebo
cd catkin_ws
catkin_init_workspace
catkin build
```
### Test
Connection(MAVROS)
```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

Start simulation
```
cd ~/UAV_Swarm_gazebo/Autopilot
make px4_sitl gazebo_ral_x6
```
Run ros node
```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash
rosrun takeoff_pkg takeoff_node
```
### Multi
```
cd UAV_Swarm_gazebo/Autopilot/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 multi_plane_line.launch vehicle:=ral_x6 input:=4
```
```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash
rosrun path_ta better_scout_0.py
```

