# [Tutorial] UR5 Simulation on Gazebo Environment

## [Ref]
- 1. https://roboticscasual.com/ros-tutorial-simulate-ur5-robot-in-gazebo-urdf-explained/

## [DO]
### 1. Run Gazebo
- Terminal에서 Gazebo 
```
curl -sSL http://get.gazebosim.org | sh

```

- 다른 terminal에서 roslaunch
```
roslaunch gazebo_ros empty_world.launch
```

### 2. 

- ROS distro가 noetic이므로 
- universal_robot package from the ros industrial project which contains the visual and geometric description files of the ur5 robot. If you did one of the previous tutorials you might have it already installed
```
git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src
```

- 그리고 build
```
~/catkin_ws$ catkin_make
~/catkin_ws$ source devel/setup.bash
```

## 3. Launching the UR5 in Gazebo with ros_control

Now let’s start up an empty world in gazebo as before:
```
roslaunch gazebo_ros empty_world.launch
```

In a new terminal we can spawn the robot with the spawn_model command:

```
rosrun gazebo_ros spawn_model -file /<path-to-your-gazebo-urdf-file>.urdf -urdf -x 0 -y 0 -z 0.1 -model ur5
```

- 로봇이 스폰될 좌표와 URDF 파일의 경로를 인수로 전달합니다. 빈 월드의 맨 중앙에 모델 아래 왼쪽에도 나타나는 접지판이 있기 때문에 Z축에 0.1의 작은 오프셋을 정의합니다. 이 플레이트는 보이지 않지만 로봇을 같은 위치에 정확히 배치하려고 할 때 충돌을 일으킵니다.