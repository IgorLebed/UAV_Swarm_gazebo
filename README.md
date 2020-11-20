# UAV_Swarm_gazebo

## Introduction

## Properation


```
cd $HOME
mkdir src
```
```
cd ~/src
git clone https://github.com/IgorLebed/PX4-Autopilot.git
cd PX4-Autopilot/
bash ./Tools/setup/ubuntu.sh
```
```
test launching 

make px4_sitl gazebo_iris
```
```
cd ~/src
mkdir -p catkin_ws/src
cd catkin_ws
catkin_init_workspace
cd src
git clone https://github.com/IgorLebed/mavros.git
git clone https://github.com/IgorLebed/mavlink.git
cd ..
catkin build
```
```

