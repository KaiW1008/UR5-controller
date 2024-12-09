#!/usr/bin/env python

from API.robot_API import UR5MotionAPI
import rospy

if __name__ == "__main__":
    try:
        rospy.init_node('ur5_motion_control', anonymous=True)
        # rospy.sleep(2)
        # Initialize the API
        api = UR5MotionAPI()

        # Move in joint space
        point1 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        point2 = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
        api.move_to_joint_positions(point1, point2, velocity=0.25, acceleration=0.1)
        
        rospy.sleep(2)

        # Move in Cartesian space
        start_xyz = [0.5, 0, 0.5]
        end_xyz = [0.7, -0.2, 0.3]
        api.move_to_cartesian_positions(start_xyz, end_xyz, linear_velocity=0.1, linear_acceleration=0.1)
        
        rospy.sleep(2)

        # Get robot state
        state = api.get_robot_state()
        if state:
            rospy.loginfo("Current Robot State: %s", state)


        rospy.sleep(2)
        rospy.loginfo("All motions executed successfully.")
    except rospy.ROSInterruptException:
        rospy.loginfo("UR5 Motion API Node Terminated.")


