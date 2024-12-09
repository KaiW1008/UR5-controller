
## **Overview**

This project provides a simulation environment for controlling a UR5 robot in ROS-Gazebo. It includes:
1. A **motion library** (`ur5_motion_library`) for generating joint and Cartesian motions.
2. An **API** (`UR5MotionAPI`) for high-level robot control and state retrieval.
3. A **Co-Pilot** interface powered by an open-source LLM (`gpt-neo-125M`) to generate Python code suggestions based on natural language descriptions.

---

## **Features**

### Motion Library
- **Joint-Space Motion**: Moves the robot between two joint configurations with specified velocity and acceleration.
- **Cartesian-Space Motion**: Moves the robot between two Cartesian poses using an inverse kinematics solver.

### UR5MotionAPI
- Provides user-friendly functions to:
  - Move the robot in joint space or Cartesian space.
  - Retrieve the current robot state.

### Co-Pilot Interface
- Uses an open-source LLM to suggest Python code for robot control.
- Accepts natural language task descriptions and generates Python scripts.

---

## **File Structure**

