
sudo pip install pyquaternion


### run pixhawk connection(MAVROS)

```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### or run pixhawk&gazebo

```
cd UAV_Swarm_gazebo/Autopilot/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 multi_uav_mavros_sitl.launch vehicle:=ral_x6
```

## switch pixhawk to takeoff mode

```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash
rosrun offb offb_node
```
