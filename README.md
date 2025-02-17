# mycobot_ros2 #
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)



## Quick Start

Froked from [here](https://github.com/automaticaddison/mycobot_ros2)



Understood! I will extract the crucial steps from all 12 tutorials and consolidate them into a step-by-step guide in a README file. This will allow you to copy and paste the commands, ensuring you've effectively gone through all 12 demos without manually reading each tutorial. 

I'll include only the necessary launch commands and steps, along with key information from each tutorial. Once it's ready, I'll provide the README file for you to use.

# MoveIt 2 & ROS 2 Jazzy – Quick-Start Tutorial (12 Demos)

This README provides **step-by-step commands** to run 12 MoveIt 2 tutorials (ROS 2 "Jazzy"). Each section highlights the minimal steps to launch the demo and notes any special requirements or differences. You can copy-paste these commands to quickly get each demo running. 

**Prerequisite:** Make sure you have cloned the project repository and built the workspace (including any required dependencies like ROS 2 Control, MoveIt 2, and MoveIt Task Constructor). For example: 

```bash
# Clone the repository and checkout the Jazzy branch
git clone -b jazzy https://github.com/automaticaddison/mycobot_ros2.git ~/ros2_ws/src/mycobot_ros2

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build
source ~/.bashrc
```

----

## 1. Simulating a Robotic Arm in Gazebo

**Goal:** Launch the myCobot 280 robotic arm in Gazebo with ROS 2 controllers. Verify that you can command the arm and gripper using ROS 2 topics/actions.

**Steps:**

1. **Start Gazebo Simulation:** Open a terminal and launch the Gazebo simulation with the myCobot arm and controllers: 

   ```bash
   ros2 launch mycobot_gazebo mycobot_280_arduino_bringup_ros2_control_gazebo.launch.py
   ``` 

   This spawns the robot in Gazebo and loads the joint controllers (you should see the arm in an empty world) ([How to Simulate a Robotic Arm in Gazebo – ROS 2 Jazzy](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/#:~:text=echo%20,y%3A%3D0.0)) ([How to Simulate a Robotic Arm in Gazebo – ROS 2 Jazzy](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/#:~:text=Now%20let%E2%80%99s%20launch%20everything)).

2. **Control the Gripper (Optional):** You can test the gripper controller using ROS 2 action commands. For example, to **close** the gripper: 

   ```bash
   ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand \
   "{command: {position: -0.7, max_effort: 5.0}}"
   ``` 

   And to **open** the gripper: 

   ```bash
   ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand \
   "{command: {position: 0.0, max_effort: 5.0}}"
   ``` 

   These commands send a position to the gripper (in this case, -0.7 for closed, 0.0 for open) ([How to Simulate a Robotic Arm in Gazebo – ROS 2 Jazzy](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/#:~:text=If%20you%20want%20to%20close,gripper%2C%20you%20can%20do%20this)).

3. **Move the Arm (Optional):** You can command a joint trajectory to move the arm. For example, send the arm to a new joint configuration: 

   ```bash
   ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
     joint_names: ['link1_to_link2','link2_to_link3','link3_to_link4','link4_to_link5','link5_to_link6','link6_to_link6_flange'],
     points: [{ positions: [1.345, -1.23, 0.264, -0.296, 0.389, -1.5], time_from_start: {sec: 3.0}}]
   }"
   ``` 

   This single command (formatted on multiple lines for readability) publishes a trajectory for the six joints ([How to Simulate a Robotic Arm in Gazebo – ROS 2 Jazzy](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/#:~:text=Here%20is%20a%20command%20you,is%20all%20a%20single%20command)). The arm should move to the specified joint angles over 3 seconds. You can similarly publish a trajectory of all zeros to return the arm to its home position ([How to Simulate a Robotic Arm in Gazebo – ROS 2 Jazzy](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/#:~:text=You%20can%20also%20move%20it,back%20home%20now)).

**Key Points / Differences:** This demo uses **ROS 2 Control** to simulate the arm. Make sure the controllers (`arm_controller`, `gripper_action_controller`, etc.) are active (you can check with `ros2 control list_controllers` if needed) ([How to Simulate a Robotic Arm in Gazebo – ROS 2 Jazzy](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/#:~:text=See%20the%20active%20controllers%3A)). In this step, we are **not** using MoveIt 2 yet – all motions are commanded directly via ROS 2 topics/actions.

----

## 2. Configuring MoveIt 2 for a Simulated Robot Arm

**Goal:** Launch the MoveIt 2 motion planning framework and RViz for the simulated arm. This will allow planning trajectories using MoveIt (OMPL by default, plus Pilz and STOMP configured).

**Steps:**

1. **Start Gazebo (if not already running):** In a terminal, launch the simulation as in Tutorial 1. If you still have Gazebo running from the previous step, you can reuse it. Otherwise, start it again:

   ```bash
   ros2 launch mycobot_gazebo mycobot_280_arduino_bringup_ros2_control_gazebo.launch.py use_rviz:=false
   ``` 

   (We disable RViz here because MoveIt will launch its own RViz session) ([How to Configure MoveIt 2 for a Simulated Robot Arm](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#:~:text=ros2%20launch%20mycobot_gazebo%20mycobot_280_arduino_bringup_ros2_control_gazebo,py)).

2. **Launch MoveIt and RViz:** Open a **new terminal** (keep Gazebo running) and launch the MoveIt `move_group` node with RViz: 

   ```bash
   ros2 launch mycobot_moveit_config move_group.launch.py
   ``` 

   This starts the MoveIt motion planning server and opens RViz with the MoveIt configuration ([How to Configure MoveIt 2 for a Simulated Robot Arm](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#:~:text=Launch%20MoveIt%20and%20RViz)). You should see the robot in RViz along with the MoveIt Motion Planning plugin.

3. **Plan a Motion in RViz:** In RViz, use the "Motion Planning" panel to plan and execute a trajectory:
   - Select the "Interactive Marker" and move it to set a target pose for the robot’s end effector.
   - Click **Plan** and then **Execute**. The arm in Gazebo should follow the planned trajectory. 

   MoveIt is now controlling the arm via the ROS 2 controllers. By default, the **Pilz Industrial Planner** is set as the planning pipeline (see Tutorial 3 for how to switch planners) ([How to Configure MoveIt 2 for a Simulated Robot Arm](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#:~:text=.planning_pipelines%28%20pipelines%3D%5B,publish_robot_description%3DFalse%2C%20publish_robot_description_semantic%3DTrue%2C%20publish_planning_scene%3DTrue%2C)) ([How to Configure MoveIt 2 for a Simulated Robot Arm](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#:~:text=This%20launch%20file%20launches%20MoveIt,and%20RViz)).

**Key Points:** Ensure the **Gazebo simulation is running** before launching MoveIt; otherwise, the MoveIt RViz might not find the robot’s state topics and the planners could time out. The MoveIt launch above uses simulation time (`use_sim_time`) so everything stays synchronized with Gazebo ([How to Configure MoveIt 2 for a Simulated Robot Arm](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#:~:text=moveit_config.to_dict%28%29%2C%20,%5D%2C)). At this stage, MoveIt’s planning pipeline has Pilz, STOMP, and OMPL available (configured in the launch). You can now use MoveIt’s interactive tools to plan collision-free paths.

----

## 3. Comparing Pilz, STOMP, and OMPL Planners

**Goal:** Understand the differences between the Pilz, STOMP, and OMPL planners and how to use each for motion planning. (This is a conceptual tutorial – the robot setup is the same as in Tutorial 2.)

**Steps/Notes:**

- **Pilz Industrial Motion Planner:** A deterministic planner ideal for simple, constrained motions (linear, circular, or PTP moves) where speed and predictability are key ([Difference between Pilz, STOMP, and OMPL Planners](https://automaticaddison.com/difference-between-pilz-stomp-and-ompl-planners/#:~:text=When%20to%20use%20Pilz)) ([Difference between Pilz, STOMP, and OMPL Planners](https://automaticaddison.com/difference-between-pilz-stomp-and-ompl-planners/#:~:text=Use%20Pilz%20when%20your%20cooking,stirring%20motion%20in%20a%20bowl)). *When to use:* repetitive or industrial tasks like a straight-line approach or a fixed stirring motion – Pilz ensures consistent and fast execution.

- **STOMP (Stochastic Trajectory Optimization for Motion Planning):** An optimization-based planner that improves trajectory smoothness and can escape local minima due to its stochastic nature ([Difference between Pilz, STOMP, and OMPL Planners](https://automaticaddison.com/difference-between-pilz-stomp-and-ompl-planners/#:~:text=When%20programming%20a%20robotic%20arm,each%20one%20over%20the%20others)) ([Difference between Pilz, STOMP, and OMPL Planners](https://automaticaddison.com/difference-between-pilz-stomp-and-ompl-planners/#:~:text=When%20to%20use%20STOMP)). *When to use:* if you need a smooth path or have path constraints (e.g., maintain a certain orientation) – STOMP trades extra computation time for a potentially higher-quality path.

- **OMPL (Open Motion Planning Library):** A set of sampling-based planners (default in MoveIt) good for complex, high-dimensional planning problems. *When to use:* for general-purpose planning in cluttered environments or when no special constraints require Pilz or STOMP – OMPL offers flexibility and a variety of planning algorithms.

- **Selecting a Planner:** In the MoveIt RViz Motion Planning panel, you can choose the planner. By default, the MoveIt config uses Pilz as the pipeline ([How to Configure MoveIt 2 for a Simulated Robot Arm](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#:~:text=.planning_pipelines%28%20pipelines%3D%5B,publish_robot_description%3DFalse%2C%20publish_robot_description_semantic%3DTrue%2C%20publish_planning_scene%3DTrue%2C)). To try OMPL or STOMP, open the **Planning** tab in RViz and look for the planner/pipeline selection drop-down (it may list OMPL, Pilz, STOMP if configured). Select the desired planner and click **Plan** again to see how the trajectory or planning time changes. 

No additional launch commands are needed beyond what was done in Tutorial 2. This step is about exploring the planners:
  - For example, try planning a move with **Pilz** (notice it may choose a simple linear path) versus **OMPL** (which might take a slightly different path or longer planning time if obstacles are present).
  - If STOMP is installed and configured, you can set it as the planner and plan; STOMP may produce a smoother path if one exists, at the cost of more planning time.

**Key Points:** Pilz is **fast and deterministic** (great for known simple motions) ([Difference between Pilz, STOMP, and OMPL Planners](https://automaticaddison.com/difference-between-pilz-stomp-and-ompl-planners/#:~:text=,don%E2%80%99t%20require%20complex%20trajectory%20planning)), STOMP focuses on **trajectory optimization**, and OMPL is a **general framework** that includes many planners (RRT, PRM, etc.). In practice, you choose based on your task needs. MoveIt 2 allows configuring multiple pipelines; in our setup, all three are available, and you can switch as needed without restarting Gazebo ([How to Configure MoveIt 2 for a Simulated Robot Arm](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#:~:text=.planning_pipelines%28%20pipelines%3D%5B,publish_robot_description%3DFalse%2C%20publish_robot_description_semantic%3DTrue%2C%20publish_planning_scene%3DTrue%2C)). 

----

## 4. First MoveIt 2 C++ Program (Hello MoveIt)

**Goal:** Run a simple C++ program that uses the MoveIt C++ API (MoveGroupInterface) to move the robot to a target pose. This demonstrates programmatic motion planning and execution.

**Steps:**

1. **Ensure Simulation and MoveIt are Running:** If you closed the previous sessions, relaunch Gazebo and MoveIt:
   - Terminal 1: `ros2 launch mycobot_gazebo mycobot_280_arduino_bringup_ros2_control_gazebo.launch.py`  
   - Terminal 2: `ros2 launch mycobot_moveit_config move_group.launch.py`

   > *Tip:* If you still have them running from tutorial 2, you can reuse those.

2. **Run the C++ Demo Node:** Open another terminal and run the **hello_moveit** node:

   ```bash
   ros2 run mycobot_moveit_demos hello_moveit
   ``` 

   This will execute the C++ program which plans a motion for the arm and then executes it ([How to Create Your First C++ MoveIt 2 Project – ROS 2 Jazzy](https://automaticaddison.com/how-to-create-your-first-c-moveit-2-project-ros-2-jazzy/#:~:text=match%20at%20L251%20ros2%20run,mycobot_moveit_demos%20hello_moveit)). The program is configured to move the robot’s end-effector (gripper) to a predefined target pose in front of the robot, using the MoveIt `MoveGroupInterface` to plan and execute the trajectory.

3. **Observe the Motion:** In RViz and Gazebo, you should see the arm move from its starting "home" position to the target pose (and possibly back, depending on the code logic). The terminal running `hello_moveit` will print out status information about planning and execution. 

   *Note:* The node will wait up to 10 seconds for MoveIt / `move_group` to be available. If you run it too early (before launching MoveIt), it will error out after waiting ([How to Create Your First C++ MoveIt 2 Project – ROS 2 Jazzy](https://automaticaddison.com/how-to-create-your-first-c-moveit-2-project-ros-2-jazzy/#:~:text=match%20at%20L268%20ros2%20run,tf2_ros%20tf2_echo%20base_link%20gripper_base)). Ensure MoveIt is running (`/move_group` node is active) before executing the `ros2 run` command.

**Key Points:** This demo uses the MoveIt C++ API to request a motion plan and execute it on the simulated robot. It demonstrates a simple pick-and-place style "go to pose" command in code. We added the `mycobot_moveit_demos` package for these custom nodes (which includes `hello_moveit` as created in the tutorial) ([How to Create Your First C++ MoveIt 2 Project – ROS 2 Jazzy](https://automaticaddison.com/how-to-create-your-first-c-moveit-2-project-ros-2-jazzy/#:~:text=ros2%20pkg%20create%20,name%20hello_moveit%20%5C%20mycobot_moveit_demos)). After running, you can also verify the end effector’s final pose via a tf echo or in RViz (the tutorial suggests `ros2 run tf2_ros tf2_echo base_link gripper_base` to see the transform) ([How to Create Your First C++ MoveIt 2 Project – ROS 2 Jazzy](https://automaticaddison.com/how-to-create-your-first-c-moveit-2-project-ros-2-jazzy/#:~:text=ros2%20run%20tf2_ros%20tf2_echo%20base_link,gripper_base)).

----

## 5. Planning Around Objects (MoveIt + Collision Objects)

**Goal:** Run a C++ demo where the robot plans a path that avoids a virtual obstacle in the scene. This shows how to add collision objects to the planning scene and plan around them.

**Steps:**

1. **Launch Simulation and MoveIt:** As before, ensure Gazebo and MoveIt are running (see tutorial 4 steps). For a fresh start:
   - Terminal 1: `ros2 launch mycobot_gazebo mycobot_280_arduino_bringup_ros2_control_gazebo.launch.py`  
   - Terminal 2: `ros2 launch mycobot_moveit_config move_group.launch.py`

2. **Run the Planning Demo:** Open a new terminal and run the **plan_around_objects** node:

   ```bash
   ros2 run mycobot_moveit_demos plan_around_objects
   ``` 

   This program will add a collision object (e.g., a box or cylinder) into MoveIt’s planning scene, then plan a motion for the arm to reach a target on the other side of the object ([Add and Plan Around Objects Using MoveIt 2 – ROS 2 Jazzy](https://automaticaddison.com/add-and-plan-around-objects-using-moveit-2-ros-2-jazzy/#:~:text=ros2%20run%20mycobot_moveit_demos%20plan_around_objects)). The MoveIt Visual Tools library is used to visualize the scene and trajectory in RViz (you may see markers or messages in RViz confirming the object addition).

3. **Observe in RViz:** In RViz, you should see the obstacle appear (usually as a colored shape) and the planned trajectory drawn, avoiding the object. The program likely also triggers execution; the arm in Gazebo should move, taking a path around the virtual object. 

   If the program doesn’t auto-execute, you can manually execute the trajectory: look for a prompt or use the RViz MotionPlanning panel to execute the last planned path. 

**Key Points:** This demo introduces the **Planning Scene Interface**. The code programmatically added a collision object into the environment and then planned a path to a goal while avoiding it ([Add and Plan Around Objects Using MoveIt 2 – ROS 2 Jazzy](https://automaticaddison.com/add-and-plan-around-objects-using-moveit-2-ros-2-jazzy/#:~:text=In%20this%20tutorial%2C%20we%20will,movements%20while%20avoiding%20these%20obstacles)) ([Add and Plan Around Objects Using MoveIt 2 – ROS 2 Jazzy](https://automaticaddison.com/add-and-plan-around-objects-using-moveit-2-ros-2-jazzy/#:~:text=,Build%20the%20Package)). In practice, you can use this method to make the robot avoid known obstacles. The `moveit_visual_tools` dependency was added to publish helpful visualizations in RViz ([Add and Plan Around Objects Using MoveIt 2 – ROS 2 Jazzy](https://automaticaddison.com/add-and-plan-around-objects-using-moveit-2-ros-2-jazzy/#:~:text=cd%20)) ([Add and Plan Around Objects Using MoveIt 2 – ROS 2 Jazzy](https://automaticaddison.com/add-and-plan-around-objects-using-moveit-2-ros-2-jazzy/#:~:text=Add%20this%3A)). After running, you might see the arm take a detour around the object instead of a straight line, confirming the obstacle avoidance.

----

## 6. Setting Up MoveIt 2 Task Constructor (MTC)

**Goal:** Install and configure the MoveIt Task Constructor framework for more advanced task planning. This step sets up the environment for running Task Constructor demos (tutorials 7–12).

**Steps:**

1. **Install MoveIt Task Constructor (if not already installed):** MTC is an add-on to MoveIt. If you haven’t installed it:
   - Install MongoDB (used by MoveIt for logging/planning scene storage):  
     ```bash
     sudo apt-get update && sudo apt-get install -y mongodb-org
     ``` 
   - Install MoveIt Task Constructor packages. For ROS 2 Jazzy (Rolling), you may build from source. (In the tutorial, a specific commit was used ([How to Set Up the MoveIt 2 Task Constructor – ROS 2 Jazzy](https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/#:~:text=cd%20)).) If binary packages are available, you can try:  
     ```bash
     sudo apt-get install ros-$ROS_DISTRO-moveit-task-constructor
     ``` 
     *(Replace `$ROS_DISTRO` with your ROS 2 distribution name.)*  
     
   - After installation, rebuild your workspace to integrate MTC:  
     ```bash
     colcon build --event-handlers console_cohesion+ 
     source ~/.bashrc
     ```
     Ensure no build errors. (If building from source, refer to tutorial instructions for patching any issues ([How to Set Up the MoveIt 2 Task Constructor – ROS 2 Jazzy](https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/#:~:text=cd%20)) ([How to Set Up the MoveIt 2 Task Constructor – ROS 2 Jazzy](https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/#:~:text=moveit%3A%3Acore%3A%3AJumpThreshold%28props.get%3Cdouble%3E%28)).)

2. **Create the MTC Demo Package:** The repository already includes `mycobot_mtc_demos`, which contains sample Task Constructor demos. Ensure it’s built (it was added in the previous step). This package is configured to use MTC and has its own RViz config file (`mtc_demos.rviz`) for visualizing task pipelines ([How to Set Up the MoveIt 2 Task Constructor – ROS 2 Jazzy](https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/#:~:text=sleep%2015%20ros2%20launch%20mycobot_moveit_config,rviz_config_package%3A%3Dmycobot_mtc_demos)) ([How to Set Up the MoveIt 2 Task Constructor – ROS 2 Jazzy](https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/#:~:text=Now%20add%20an%20alias%20called,mtc_demos%20to%20your%20bashrc%20file)).

3. **Launch Gazebo and MoveIt for MTC:** We will use a slightly different launch to include the camera and a world file, as later demos involve perception. In a terminal, launch Gazebo with the **perception world** and the depth camera:

   ```bash
   ros2 launch mycobot_gazebo mycobot.gazebo.launch.py load_controllers:=true \
       world_file:=pick_and_place_demo.world use_camera:=true use_rviz:=false use_robot_state_pub:=true
   ``` 

   This uses `pick_and_place_demo.world` (which contains a table and objects for the final demo) and activates a simulated RealSense depth camera on the robot ([How to Set Up the MoveIt 2 Task Constructor – ROS 2 Jazzy](https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/#:~:text=echo%20,y%3A%3D0.0)) ([How to Set Up the MoveIt 2 Task Constructor – ROS 2 Jazzy](https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/#:~:text=ros2%20launch%20mycobot_gazebo%20mycobot.gazebo.launch.py%20,x%3A%3D0.0)). Wait for Gazebo to fully load the robot and environment.

4. **Launch MoveIt with MTC Configuration:** In a new terminal, launch MoveIt’s `move_group` with the Task Constructor RViz configuration:

   ```bash
   ros2 launch mycobot_moveit_config move_group.launch.py rviz_config_file:=mtc_demos.rviz rviz_config_package:=mycobot_mtc_demos
   ``` 

   This opens RViz with a panel called **“Motion Planning Tasks”** pre-loaded (the MTC RViz plugin) ([How to Set Up the MoveIt 2 Task Constructor – ROS 2 Jazzy](https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/#:~:text=z%3A%3D0.05%20,0)). MoveIt is running as before, and RViz is now ready to visualize Task Constructor tasks and their stages.

   > **Note:** The provided RViz config will automatically shut down everything when you close RViz (it sets RViz to cause a shutdown on exit) ([How to Configure MoveIt 2 for a Simulated Robot Arm](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#:~:text=,d%22%2C%20rviz_config_file%5D%2C%20output%3D%22screen%22%2C%20parameters%3D%5B%20moveit_config.robot_description)) ([How to Configure MoveIt 2 for a Simulated Robot Arm](https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#:~:text=exit_event_handler%20%3D%20RegisterEventHandler,reason%3D%27rviz%20exited%27%29%29%2C%20%29%2C)). Keep RViz open while running the demos.

5. **(Optional) Use Helper Script:** The tutorial provided a quick-launch script `mycobot_280_mtc_demos.sh` and suggested adding an alias `mtc_demos` for convenience ([How to Set Up the MoveIt 2 Task Constructor – ROS 2 Jazzy](https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/#:~:text=Now%20add%20an%20alias%20called,mtc_demos%20to%20your%20bashrc%20file)). If you set that up, you can launch the above two steps with one command:
   ```bash
   mtc_demos
   ``` 
   followed by the demo name (as we will use below). Otherwise, just keep Gazebo and MoveIt running from steps 3 and 4.

**Key Points:** At this stage, we have the **simulation + MoveIt + MTC** ready. The depth camera is publishing point clouds, and the environment (table, etc.) is loaded (for use in tutorial 12). The **MoveIt Task Constructor** plugin is installed and ready, but no tasks are running yet. The following tutorials (7–11) will run individual Task Constructor demo programs. Each program will construct a task (sequence of stages) and display solutions in the RViz "Motion Planning Tasks" panel.

----

## 7. Alternative Path Planning (MTC Demo)

**Goal:** Demonstrate planning the same start and goal with different optimization criteria (path length, time, end-effector motion, etc.) in parallel, using MTC’s **Alternatives** container. The robot will find multiple solution paths optimized for different metrics.

**Steps:**

1. **Launch Environment:** If not already done, follow Tutorial 6 to start Gazebo with the `pick_and_place_demo.world` and MoveIt+RViz with MTC config. Ensure RViz (with "Motion Planning Tasks" panel) is open.

2. **Run the Alternative Paths Demo:** In a new terminal, run the `alternative_path_costs` node:

   ```bash
   ros2 run mycobot_mtc_demos alternative_path_costs
   ```

   This MTC program will create an MTC **Task** with a start state and goal state, and an **Alternatives** stage containing four planning pipelines (each with a different cost term). It will ask all four planners to solve the motion and return their solutions.

3. **Inspect Solutions in RViz:** In the RViz "Motion Planning Tasks" panel, you will see a task named "alternative path costs" with multiple solutions beneath it. Click on the task and then on each solution to visualize it. Each solution corresponds to one optimization:
   - Path length minimized
   - Trajectory duration minimized
   - End-effector motion minimized
   - Elbow motion minimized

   They should all reach the same goal but via slightly different trajectories. For example, one solution might take a straighter but slower path, another a quicker but longer path, etc. Green checkmarks indicate successful solutions for each strategy. 

4. **Execute if Desired:** You can execute a particular solution by selecting it and clicking the **“Execute Solution”** button in the RViz panel. The robot in Gazebo will then follow that trajectory. (All found solutions are collision-free; you can pick any to run.)

**Key Points:** This demo shows the power of MTC to evaluate multiple strategies in parallel. The code set up multiple planners with different **cost terms** (via `CostTerm` in MTC) for the same connect stage. We effectively ran Pilz vs. OMPL vs. two OMPL variants in one go. In the RViz Task panel, you can see each strategy’s planning time and success status. **Alternatives** container in MTC allows branching the task into parallel tracks and is useful for comparing outcomes or providing fallback options (as we’ll see next).

----

## 8. Cartesian Path Planning (MTC Demo)

**Goal:** Plan a sequence of Cartesian moves (linear motions and rotations) using MoveIt Task Constructor. The demo moves the robot in a straight line in X, then in -Y, then rotates the wrist, then adjusts two joints, and finally returns to the start, illustrating a **multi-stage Cartesian path**.

**Steps:**

1. **Launch Environment:** Make sure Gazebo + MoveIt with MTC (RViz) are running (see Tutorial 6). If continuing from tutorial 7, you can simply terminate the `alternative_path_costs` node (Ctrl+C) and proceed.

2. **Run the Cartesian Path Demo:** In a terminal, run:

   ```bash
   ros2 run mycobot_mtc_demos cartesian
   ```

   This runs the `cartesian.cpp` MTC program. It creates a task with a series of fixed Cartesian motions:
   - Move end-effector +5 cm in X
   - Move end-effector –2 cm in Y
   - Rotate end-effector –18° around Z
   - Apply a small joint offset to specific joints
   - Connect all those stages smoothly
   - Final stage to return to "ready" pose

3. **Inspect the Task in RViz:** In the "Motion Planning Tasks" panel, you’ll see a task (likely named "Cartesian Path"). Expand it to view stages: "initial state", each of the moves ("x +0.05", "y -0.02", "rz -18°", "joint offset"), "connect", and "final state". All stages should have a green check (1 solution each) if planning succeeded. 

   Click on each stage to see its effect:
   - You can click through each sub-motion to see the incremental movement in RViz.
   - The **connect** stage ensures the transitions between those discrete motions are feasible (it computes joint-space interpolation to link them).

4. **Execute the Plan:** If you want, execute the entire sequence by clicking **"Execute"** in RViz (upper-right of the Task panel). The robot will perform the series of Cartesian motions in Gazebo. You should see it move in a straight line, then sideways, then twist, etc., and finally come back to the start.

**Key Points:** This demo showcases how to break down a complex trajectory into **Cartesian segments** using MTC’s `CartesianPath` solver for precise control of end-effector motion. The task combined multiple linear movements and a rotation, which MTC connected seamlessly. In a real scenario, you might use this for constrained moves (like drawing a shape or inserting a part). The ability to inspect each stage in RViz is very useful for debugging and understanding the motion plan structure.

----

## 9. Fallback Strategies (MTC Demo)

**Goal:** Demonstrate a task with **fallback planning strategies**. The robot is given multiple initial states and multiple planner options so that if one planner fails, another can solve the motion. This shows how to ensure success by trying simpler plans first and falling back to more complex ones.

**Steps:**

1. **Launch Environment:** Gazebo + MoveIt (RViz) with MTC should be running. (If you continued from a previous demo, stop its node. Otherwise, start as per Tutorial 6.)

2. **Run the Fallbacks Demo:** In a terminal, run:

   ```bash
   ros2 run mycobot_mtc_demos fallbacks_move_to
   ```

   This will execute the `fallbacks_move_to.cpp` MTC program. The task is set up with:
   - Three possible **initial states** for the robot (slightly different starting configurations).
   - One goal (a target pose on the other side of an obstacle).
   - A **Parallel** container that tries three planning methods to reach the goal: 
     1. Cartesian path planner,
     2. Pilz planner,
     3. OMPL planner.

   Each initial state will attempt each planner, in order, until one succeeds.

3. **Observe Task Results in RViz:** In the Task panel, find the task (named something like "fallback strategies in MoveTo"). Expand it to see two branches: **initial states** and **move to other side** .
   - Under "initial states", you’ll see three states listed (these are the 3 start positions considered). All should be checked (meaning each start state was evaluated).
   - Under "move to other side", you’ll see the three planners:
     - Cartesian path (likely failed for all starts – indicated by red X and 0 solutions),
     - Pilz path (succeeded for some of the starts),
     - OMPL path (succeeded for the remaining start(s)).

   This shows that for simpler cases the Pilz planner found solutions, but when an obstacle made the path complex, Pilz failed and OMPL succeeded. The fallback logic ensured the task overall found a solution for every initial state by trying planners in sequence.

4. **Visualize and Execute:** Click on the different branches and solutions to see the planned trajectories. You can choose a successful solution (green check) and execute it if desired. For example, one initial state might be directly reachable (Pilz solves it), another might require going around an obstacle (only OMPL finds a path). Executing will make the robot move accordingly in Gazebo.

**Key Points:** This demo is about robustness. The MTC task used a **Fallback** pattern: it tried a fast, simple planner first (Cartesian, which failed in collision), then a slightly more capable one (Pilz, which handled some cases), and finally the general one (OMPL for the hardest case). It also considered multiple possible starts (perhaps the robot could start in different orientations). MTC processed all and found at least one valid trajectory for each scenario. This approach is useful in dynamic environments: try the quickest plan, if it fails, automatically fall back to a more powerful planner. In our visualization, we confirmed:
- Cartesian planner yielded 0 solutions (expected when an obstacle is in the direct path) .
- Pilz solved 2/3 cases, OMPL solved the remaining 1/3

----

## 10. Inverse Kinematics with Clearance (MTC Demo)

**Goal:** Use the MTC **ComputeIK** stage to find inverse kinematic solutions for a target pose, with a custom cost term favoring solutions that keep the robot farther from obstacles. Essentially, the robot will compute multiple IK solutions for a pose and rank them by clearance from a nearby obstacle.

**Steps:**

1. **Launch Environment:** Start Gazebo + MoveIt with MTC as before (Tutorial 6). The `pick_and_place_demo.world` should be loaded, as this demo expects a spherical obstacle in the scene.

2. **Run the IK Clearance Demo:** In a terminal, run:

   ```bash
   ros2 run mycobot_mtc_demos ik_clearance_cost
   ```

   This executes the `ik_clearance_cost.cpp` MTC program. The task does the following:
   - Adds a **spherical obstacle** into the planning scene (if not already present).
   - Defines a target pose for the robot’s gripper.
   - Uses an MTC **ComputeIK** stage to compute all possible IK solutions (robot joint configurations) that reach that target pose.
   - Attaches a custom **clearance cost** to each IK solution, which penalizes being close to the obstacle.
   - The task outputs the IK solutions, sorted by cost (clearance).

3. **View Results in RViz:** In the Task panel, you’ll see a task (likely named "clearance IK"). It should show two main stages: "IK" and "initial state". The IK stage will have found one or more solutions:
   - In the example, it found 2 IK solutions for the end-effector target.
   - The panel will show a **“cost”** column for each solution – this is the clearance cost. A lower cost means the robot was closer to the obstacle, higher means it kept away. You should see the computed cost value for each solution (e.g., ~66.53 as in the tutorial) and a comment indicating the clearance distance achieved.
   - Both solutions are valid (green checkmarks) and achieve the goal; one is just more "comfortable" (farther from the obstacle) than the other.

   Select the "IK" stage to see the two possible robot configurations in RViz. They might be slightly different arm poses reaching the same point.

4. **Evaluate and Execute:** The task doesn’t automatically connect to a full motion plan here (it was focused on IK). If you want to test a full motion:
   - You could wrap these IK solutions into a full plan by adding a Connect stage to an initial state (not done in this standalone demo).
   - However, for demonstration, you can pick one IK solution and manually set it as a goal in MoveIt or just note the difference. Executing directly isn’t applicable since this demo didn’t plan a path, it only computed end configurations.

**Key Points:** This demo highlights using MTC for **IK solving with custom criteria**. Instead of letting MoveIt choose any one IK solution, we enumerated multiple and ranked them. This is useful when some IK solutions are technically valid but risky (too close to collisions). We introduced a **clearance cost term** so the Task Constructor prefers the IK solution where the robot keeps a safer distance from the obstacle. The MTC Task panel’s cost column in RViz reflects this – a higher cost means more clearance penalty (so actually closer to obstacle), whereas the lowest cost indicates the best clearance. In practice, you could feed the best IK solution into a planner like OMPL to get an actual trajectory.

----

## 11. Reusing Motion Plans (Modular Tasks, MTC Demo)

**Goal:** Build a complex task by **reusing a sequence of motions** multiple times. This demo creates a reusable module (a set of moves) and inserts it repeatedly into a larger task. The robot will perform a series of moves (module) several times in a row, demonstrating how to compose tasks modularly.

**Steps:**

1. **Launch Environment:** As usual, ensure Gazebo + MoveIt (RViz with MTC) are running (Tutorial 6). If continuing, stop the previous node.

2. **Run the Modular Task Demo:** In a terminal, run:

   ```bash
   ros2 run mycobot_mtc_demos modular
   ```

   This runs the `modular.cpp` MTC program. The program:
   - Defines a reusable **module** of movements (move +5cm X, move -2cm Y, rotate -18° Z, move to "ready").
   - Creates a main task that:
     - Starts from current state,
     - Moves to "ready" pose,
     - Executes the above module **five times** in succession ,
     - Finally moves back to "home" position .
   - Essentially, the arm will do a certain pattern 5 times.

3. **Inspect in RViz:** In the Task panel, look for the task (named "Reusable Containers" or similar). Expand it and examine the stages:
   - "current" (initial state),
   - "move to ready",
   - then you should see five repeats of a container (each might be labeled something like "Cartesian Path" since that’s the module moves),
   - and finally "move to home".

   Each of those five modules contains sub-stages ("x +0.05", "y -0.02", "rz -18°", "moveTo ready") – essentially the same sequence repeated 5 times. All should have solutions (checkmarks) and the number of solutions `#` should be 1 for each (we expect one nominal solution per stage since it’s a fixed sequence).

   This verifies that the module was inserted multiple times and planned consistently each time.

4. **Execute the Task:** Click **“Execute”** in the RViz panel to watch the robot perform the full sequence in Gazebo. It will:
   - Go to ready,
   - Do the 5-part motion sequence (which includes small moves in X, Y, Z and back to ready each time, so you might see a kind of repeated wiggle or pattern),
   - Then return to home.

   The execution should be smooth since each iteration of the module connects back to the "ready" pose, which is also the start of the next iteration (making them seamlessly chain).

**Key Points:** This demo demonstrates **modularity in task planning**. By creating a reusable sub-task ("module") and using it multiple times, we avoid duplicating code and ensure consistency. MoveIt Task Constructor allows us to treat that module as a single stage and insert it repeatedly in the Serial container. In the RViz Task panel, the repeated structure is clearly visible, which is a powerful visual confirmation of the task design. All five iterations had identical motions (as expected) and very low costs/times (they’re short moves). This approach is useful in scenarios like assembly lines or inspection routines where the same motion sequence is needed multiple times.

----

## 12. Pick-and-Place with Perception (MTC + Perception Demo)

**Goal:** Run the final integrated demo – the robot will use its depth camera to perceive an object (a cylinder) on a table, plan a pick-and-place task using MoveIt 2 Task Constructor, and execute it. This brings together perception (point cloud processing), motion planning, and the task logic for picking up an object and placing it in a bin.

**Steps:**

1. **Launch the Full Pick-&-Place Demo:** Open a terminal and run the provided script to start everything:

   ```bash
   bash ~/ros2_ws/src/mycobot_ros2/mycobot_mtc_pick_place_demo/scripts/robot.sh
   ``` 

   (If you added the alias `pick` as suggested in the tutorial, you can simply run `pick`.This script will:
   - Launch Gazebo with the `pick_and_place_demo.world` (table and object) and the depth camera,
   - Start the MoveIt `move_group` node and RViz,
   - Launch the **point cloud processing node** (`GetPlanningSceneServer`), which reads the camera’s point cloud to identify the table plane and the object,
   - Launch the **pick and place task node** with all necessary parameters loaded .

   Give it a few seconds to initialize everything. RViz will show the scene: you should see a cloud or detected shapes, and the planning scene in RViz should display a collision object (the cylinder to pick) and possibly a target drop location.

2. **Monitor Perception:** The perception node will segment the point cloud to find the dominant plane (table) and the cylindrical object on it. It will add a collision object for the table and for the cylinder into the planning scene. In RViz, check that a cyan-colored cylinder (or similar) appears where the object is on the table (this is the collision object representing what to pick).

3. **Task Execution:** The pick-and-place Task Constructor node will automatically plan the pick and place sequence:
   - Approach the object,
   - Grasp it (close gripper),
   - Lift it,
   - Move to the drop area,
   - Release it (open gripper),
   - Retreat. 

   It will likely automatically execute the plan as well. Watch Gazebo: the arm should move to the object, the gripper will close around it, then the arm moves and places the object at another location (perhaps to the side or in a container). 

   If the execution doesn’t start automatically, you can inspect the "Motion Planning Tasks" panel for the task (it might be named "Pick and Place") and manually execute the solution. However, the code is likely set to execute the trajectory programmatically once planned.

4. **Troubleshooting Notes:** 
   - You might see some warnings in the terminal – the tutorial says you can ignore the mimic joint constraint warning (Gazebo physics limitation) and an InteractiveMarkerDisplay warning.
   - If the robot has trouble picking up the object (e.g., knocking it or not gripping well), physics engine tuning might be needed. The tutorial suggests experimenting with physics engines if the grasp is unstable.
   - The camera tilt angle in the URDF can affect perception quality. In this demo, the camera is mounted looking down at the table; changing its angle can change how well the object is detected.

5. **(Optional) View Raw Point Cloud:** To see what the robot’s camera sees, run the `pointcloud` script in another terminal (if alias added, just type `pointcloud`) . This opens RViz with a configuration to display the raw point cloud from the depth camera. This is purely for visualization and not required for the pick/place task to run.

**Key Points:** This final demo integrates everything:
- **Perception:** A pipeline using PCL identifies primitive shapes (a cylinder in this case) from the 3D point cloud and adds them as collision objects in MoveIt .
- **Planning:** An MTC task plans a pick & place: approach, grasp, lift, move, place, retreat. It uses the object’s pose from perception as input. Parameters in YAML files (e.g., grasp pose offset, approach distances, etc.) fine-tune the task behavior 
- **Execution:** The task likely auto-executes once planned. If the MTC execution issue was patched (as in tutorial 6 fixes), the robot carries out the plan autonomously. Otherwise, one could execute via RViz as a fallback (the tutorial noted a known bug where executing via code needed a fix).

When run successfully, you should see in RViz the planned grasp and place waypoints, and in Gazebo the robot physically picking up the object. The **MoveIt planning scene** in RViz will update: once the object is "attached" to the gripper, it will turn into an attached object and no longer be a collision obstacle on the table. After placing, it will be detached and left in the new location. All of this is handled by the Task Constructor sequence.

This completes the series of demos. You now have a running example of a full pick-and-place task on a simulated robot, using MoveIt 2 Task Constructor and perception – congrats!

----

