## Overview
This implementation applies a Model Predictive Path Integral (MPPI) controller to the xArm6 robot for motion control. Isaac Gym is utilized as the dynamic simulation environment for generating rollouts. The system provides an interface compatible with ROS2 Humble, enabling sim-to-real transfer and testing within a ROS-based workspace, e.g, xarm_ws, keti_ws.

# Installation

*NOTE: To use the GPU pipeline (default) you need an NVIDIA graphics card. If you do not have one, you can use the CPU pipeline, with less samples.*

This project requires the source code of IsaacGym inside the folder
`thirdparties`. Download it from https://developer.nvidia.com/isaac-gym, extract it, and place
it in `mppi-isaac/thirdparty`. Then you can proceed with the installation descibed below. 

## Virtual environment (advised)
You can install the necesary dependancies using [poetry](https://python-poetry.org/docs/) virtual environment. After installing poetry, move in `mppi-isaac` and run
```bash
poetry install --with dev
```

Access the virtual environment using
```bash
poetry shell
```

### **Test the installation**
Test that everything is properly set-up, use pytest
```bash
cd examples
poetry run pytest
```
## System-level installation
Alternatively, you can also install at the system level using pip, even though we advise using the virtual environment:
```bash
pip install .
```

## Troubleshooting
If you have an Nvidia card and after running the simulation you get a black screen, you might need to force the use of the GPU card through ``export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json``. Run this command from the same folder as the script to be launched for every terminal

# Running the examples
Access the virtual environment if installed with poetry (with `poetry shell`). You can run two types of examples. In the `examples` folder you find all the scripts. The ones realed to IsaacGym contains either "server" or "client" in the name. 

## Gazebo examples
To run the xArm6 manipulation examples with Gazebo, follow the instruction below:

### In xarm_ws repository:
Launch Gazebo simulator:
```bash 
ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py load_controller:=true joint_state_broadcaster:=true
```
Check whether the joint states are published:
```bash 
ros2 topic echo /joint_states
```
Load velocity controller:
```bash 
ros2 run controller_manager spawner xarm6_velocity_controller --controller-type velocity_controllers/JointGroupVelocityController --param-file ../xarm_ws/src/xarm_ros2/xarm_controller/config/velocity_based_controller.yaml --inactive
```
Setup default joint pose within the xArm6 joint range:
```bash 
ros2 topic pub --once /xarm6_traj_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], points: [{positions: [0.0, -0.78,-0.78, 0.0, 1.5708, 0.0], time_from_start: {sec: 5, nanosec: 0}}]}"
```
Disable trajectory based controller:
```bash 
ros2 control set_controller_state xarm6_traj_controller inactive
```
Enable velocity controller:
```bash 
ros2 control set_controller_state xarm6_velocity_controller active
```

### In mppi_isaac repository:
In the first terminal the client:
```bash 
python3 examples/xarm6_push/client.py
```
In the second terminal run the ROS2 bridge:
```bash 
python3 examples/xarm6_push/ros2_isaac_bridge.py
```
In the third terminal run the Gazebo-based server:
```bash 
python3 examples/xarm6_push/gazebo_server.py
```

## IsaacGym examples
In the first terminal the client:
```bash 
python3 examples/xarm6_push/client.py
```
In the second terminal run the IsaacGym-based server:
```bash 
python3 examples/xarm6_push/server.py
```
