## RUN

### run pixhawk connection(MAVROS)

```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### run pixhawk&gazebo

```
cd ~/UAV_Swarm_gazebo/Autopilot
make px4_sitl gazebo_ral_x6
```

### or run pixhawk&gazebo

```
cd UAV_Swarm_gazebo/Autopilot/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch vehicle:=ral_x6

roslaunch px4 posix_sitl.launch world:=$(pwd)/Tools/sitl_gazebo/worlds/ral_x6.world vehicle:=$(pwd)/Tools/sitl_gazebo/models/ral_x6/ral_x6.sdf
```

## switch pixhawk to takeoff mode

```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash
rosrun takeoff_pkg takeoff_node
```