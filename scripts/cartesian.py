#!/usr/bin/env python

from ur5_motion_library.ur5_motion_library import UR5MotionLibrary

from spatialmath import SE3
import rospy

if __name__ == "__main__":
    try:  
        # Initialize the UR5MotionLibrary
        controller = UR5MotionLibrary()

        # Joint-space motion example
        point1 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        point2 = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
        controller.move_joint_space(point1, point2, velocity=0.25, acceleration=0.1)

        # Cartesian-space motion example
        start_pose = SE3(0.5, 0, 0.5)  # Starting Cartesian pose
        end_pose = SE3(0.7, -0.2, 0.3)  # Target Cartesian pose
        controller.move_cartesian_space(start_pose, end_pose, linear_velocity=0.1, linear_acceleration=0.1, steps=100)

        rospy.loginfo("Motion commands executed.")
    except rospy.ROSInterruptException:
        rospy.loginfo("UR5 Motion Controller Node Terminated.")
