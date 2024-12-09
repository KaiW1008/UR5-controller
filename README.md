## Overview
This project provides a simulation environment for controlling a UR5 robot in ROS-Gazebo. It includes:
1. A demonstration of sin-wave movement in ROS-Gazebo
2. A **motion library** (`ur5_motion_library`) for generating joint and Cartesian motions (including demostrations). 
3. An **API** (`UR5MotionAPI`) for two control motions and state retrieval (including demostrations).
4. A **Co-Pilot** interface powered by an open-source LLM (`gpt-neo-125M`) to generate Python code suggestions based on natural language descriptions.

## Dependencies
#### Python Packages
ROS-noetic is in used for this project.
```bash
sudo apt-get install ros-noetic-universal-robots
```
Ensure the following Python packages are installed:
- `transformers`: For LLM Co-Pilot functionality. However lightweight PyTorch and Transformers are installed only for testing.
- `roboticstoolbox-python`: KDL, for inverse kinematics and motion planning.
- `spatialmath-python`: For Cartesian pose representation.
- `numpy`: For numerical operations.

Install them using pip:
```bash
pip3 install roboticstoolbox-python
pip install transformers
pip install spatialmath-python
```
lightweight PyTorch (only for testing)
```bash
dpip install torch==1.13.1+cpu torchvision==0.14.1+cpu -f https://download.pytorch.org/whl/torch_stable.html
```

## Usage

#### Sine wave movement 
```bash
roslaunch ur5_control ur5_sin_wave.launch 
```
#### Motion 1: joint movement (with library) 
```bash
roslaunch ur5_control motion1_demo.launch
```
#### Motion 2: cartesian space movement (with library) 
```bash
roslaunch ur5_control motion2_demo.launch
```
#### API demostrations (motion 1, motion2, get-robot state)
```bash
roslaunch ur5_control API_demo.launch 
```
#### co-pilot interface for motions-API
```bash
rosrun ur5_control API_LLM.py
```
-Example use: 
 Describe your task: Move the robot in joint space from [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] to [1.0, 0.5, 0.2, 0.1, 0.0, 0.0] with velocity 0.2 and acceleration 0.1

