# mycobot_ros2 #
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

Do this in every terminal
```
source /opt/ros/jazzy/setup.bash
```

```
source install/setup.bash
```


## Quick Start


Clone this repo inside the `src` of your `ros2_ws`, then go to the root of your workspace and check the dependencies by:



```
rosdep install -i --from-path src --rosdistro jazzy -y
```

If successful, then do: 

```
colcon build
```

```
source install/setup.bash
```

## To Simulate a Robotic Arm in Gazebo – ROS 2 Jazzy

[link](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/)

```
bash ~/ros2_ws/src/mycobot_ros2/mycobot_bringup/scripts/mycobot_280_gazebo.sh
```


```
ros2 run mycobot_system_tests  arm_gripper_loop_controller
```


These are the `nodes` and `controllers` running:

```
ajay@asusROG:~/ros2_ws$ ros2 node list
/arm_controller
/arm_gripper_loop_controller
/controller_manager
/gripper_action_controller
/gz_ros_control
/joint_state_broadcaster
/joint_state_publisher
/robot_state_publisher
/ros_gz_bridge
/ros_gz_image
/rviz
/transform_listener_impl_63fb3daba040
```  

```
ajay@asusROG:~/ros2_ws$ ros2 control list_controllers
gripper_action_controller position_controllers/GripperActionController           active
arm_controller            joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster   joint_state_broadcaster/JointStateBroadcaster          active

```
