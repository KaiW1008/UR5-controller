#!/usr/bin/env python

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboticstoolbox as rtb
from std_msgs.msg import Header

class UR5MotionLibrary:
    def __init__(self):
        # Initialize the Publisher to publish to the controller topic
        rospy.init_node('ur5_motion_library', anonymous=True)
        self.pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(2)
        rospy.loginfo("UR5 Motion Library Initialized.")

        # Initialize the UR5 model
        self.robot = rtb.models.UR5()

    def move_joint_space(self, point1, point2, velocity=1.0, acceleration=1.0):
        # Build the JointTrajectory message
        joint_traj = JointTrajectory()
        joint_traj.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        # Calculate duration needed to move each joint from point1 to point2 based on velocity
        joint_durations = [abs(p2 - p1) / velocity for p1, p2 in zip(point1, point2)]
        max_duration = max(joint_durations)  # Ensure all joints finish movement simultaneously

        # Create JointTrajectoryPoint for start point
        start_point = JointTrajectoryPoint()
        start_point.positions = point1
        start_point.velocities = [0.0] * 6
        start_point.accelerations = [0.0] * 6
        start_point.time_from_start = rospy.Duration(0.5)  # Starting immediately

        # Create JointTrajectoryPoint for end point
        end_point = JointTrajectoryPoint()
        end_point.positions = point2
        end_point.velocities = [velocity] * 6
        end_point.accelerations = [acceleration] * 6
        end_point.time_from_start = rospy.Duration(max_duration)  # Set the maximum duration

        # Add both points to the JointTrajectory
        joint_traj.points = [start_point, end_point]

        # Publish the message
        rospy.loginfo("Publishing joint trajectory to move from point1: %s to point2: %s", point1, point2)
        self.pub.publish(joint_traj)

        # Give enough time for the robot to reach the target position
        rospy.sleep(max_duration + 1.0)

    def move_cartesian_space(self, start_pose, end_pose, linear_velocity=0.1, linear_acceleration=0.1, steps=100):
        # Generate Cartesian path
        cartesian_path = rtb.ctraj(start_pose, end_pose, steps)
        
        cartesian_points = np.array([[T.t[0], T.t[1], T.t[2]] for T in cartesian_path])

        # Calculate inverse kinematics
        joint_trajectory = []
        for T in cartesian_path:
            q_init = joint_trajectory[-1] if joint_trajectory else np.zeros(len(self.robot.q))  # Use previous solution
            sol = self.robot.ikine_LM(T, q0=q_init)
            if sol.success:
                joint_trajectory.append(sol.q)
            else:
                rospy.logwarn("IK solution failed for pose: %s", T)
                return

        # Build JointTrajectory message
        joint_traj_msg = JointTrajectory()
        joint_traj_msg.header = Header()
        joint_traj_msg.header.stamp = rospy.Time.now()
        joint_traj_msg.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        # Calculate dynamic time intervals
        total_distance = sum(np.linalg.norm(cartesian_points[i] - cartesian_points[i - 1]) for i in range(1, len(cartesian_points)))
        time_per_step = total_distance / (linear_velocity * steps)

        for i, q in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            point.positions = q
            point.velocities = [linear_velocity] * len(q)  # Add linear velocity
            point.accelerations = [linear_acceleration] * len(q)  # Add linear acceleration
            point.time_from_start = rospy.Duration((i + 1) * time_per_step)  # Dynamic timing
            joint_traj_msg.points.append(point)

        # Publish message
        rospy.loginfo("Publishing Cartesian trajectory.")
        self.pub.publish(joint_traj_msg)
