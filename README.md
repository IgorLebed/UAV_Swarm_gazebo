# UAV_Swarm_gazebo

## Introduction

## Properation
```
cd $HOME
mkdir src
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
source ~/src/catkin_ws/devel/setup.bash
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


