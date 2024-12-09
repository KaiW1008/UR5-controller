#!/usr/bin/env python

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Function to publish sin-wave trajectory 
def sin_wave_publisher():
    
    rospy.loginfo("Sin wave publisher node started.")
    
    rospy.init_node('ur5_sin_wave_publisher', anonymous=True)
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)
    joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        current_time = rospy.get_time() - start_time

        joint_traj = JointTrajectory()
        joint_traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [math.sin(current_time)] * 6
        point.time_from_start = rospy.Duration(0.5)
       
        joint_traj.points = [point]

        pub.publish(joint_traj)
        # rospy.loginfo("Published joint trajectory point at time: %.2f with positions: %s", current_time, point.positions)        
        
        rate.sleep()


if __name__ == '__main__':
    try:
        sin_wave_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Sin wave publisher node terminated.")
        pass
