#!/usr/bin/env python

from motion_library.ur5_motion_library import UR5MotionLibrary
import rospy

if __name__ == "__main__":
    try:  
        rospy.init_node('ur5_motion_2', anonymous=True)  
        # Initialize the UR5MotionLibrary
        controller = UR5MotionLibrary()
        
        rospy.loginfo("Cartesian movement demo started.")

        # Cartesian-space motion example
        start_pose = [0.5, 0, 0.5] 
        end_pose = [0.7, -0.2, 0.3]
        linear_velocity = 0.1
        linear_acceleration = 0.1
        
        rospy.loginfo(f"Parameters: Pose1: {start_pose}, Pose2: {end_pose}, Linear velocity: {linear_velocity}, Acceleration: {linear_acceleration}")
        
        controller.move_cartesian_space(start_pose, end_pose, linear_velocity=0.1, linear_acceleration=0.1)

        rospy.loginfo("Motion commands executed.")
    except rospy.ROSInterruptException:
        rospy.loginfo("UR5 Motion Controller Node Terminated.")


