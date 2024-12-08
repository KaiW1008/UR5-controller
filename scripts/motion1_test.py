#!/usr/bin/env python

from motion_library.ur5_motion_library import UR5MotionLibrary
import rospy

if __name__ == "__main__":
    try:  
        # Initialize the UR5MotionLibrary
        rospy.init_node('ur5_motion_1', anonymous=True)  
        controller = UR5MotionLibrary()
    
        # Joint-space motion example
        velocity = 0.25
        acceleration = 0.1
        point1 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        point2 = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
        
        rospy.loginfo("Joint movement demo started.")
        rospy.loginfo(f"Parameters: Point1: {point1}, Point2: {point2}, Velocity: {velocity}, Acceleration: {acceleration}")
        
        controller.move_joint_space(point1, point2, velocity, acceleration)
        
        rospy.loginfo("Motion commands executed.")
    except rospy.ROSInterruptException:
        rospy.loginfo("UR5 Motion Controller Node Terminated.")


