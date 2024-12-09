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
roslau
```

