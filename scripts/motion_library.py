#!/usr/bin/env python

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from tf.transformations import quaternion_from_euler

class UR5MotionController:
    def __init__(self):
        # 初始化节点和Publisher
        rospy.init_node('ur5_motion_controller', anonymous=True)
        self.joint_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(2)  # 等待2秒以确保控制器完全加载
        rospy.loginfo("UR5 Motion Controller Node Initialized.")

        # 初始化MoveIt!
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")

    def move_joint_space(self, point1, point2, joints_velocity, joints_acceleration):
        # 构建JointTrajectory消息
        joint_traj = JointTrajectory()
        joint_traj.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        # 创建起始点和目标点
        # 起始点
        start_point = JointTrajectoryPoint()
        start_point.positions = point1
        start_point.velocities = [0.0] * 6
        start_point.accelerations = [0.0] * 6
        start_point.time_from_start = rospy.Duration(0.0)  # 立即开始

        # 目标点
        end_point = JointTrajectoryPoint()
        end_point.positions = point2
        end_point.velocities = [joints_velocity] * 6
        end_point.accelerations = [joints_acceleration] * 6

        # 计算运动的时间
        max_joint_displacement = max([abs(p2 - p1) for p1, p2 in zip(point1, point2)])
        movement_duration = max_joint_displacement / joints_velocity
        end_point.time_from_start = rospy.Duration(movement_duration)

        # 添加起始点和目标点到JointTrajectory
        joint_traj.points = [start_point, end_point]

        # 发布消息
        rospy.loginfo("Publishing joint trajectory from point1: %s to point2: %s", point1, point2)
        self.joint_pub.publish(joint_traj)

        # 等待运动完成
        rospy.sleep(movement_duration + 1.0)

    def move_cartesian_space(self, pose1, pose2, linear_velocity, linear_acceleration):
        # 使用MoveIt!计算笛卡尔空间运动
        waypoints = []

        # 起始姿态
        start_pose = Pose()
        start_pose.position.x = pose1.position.x
        start_pose.position.y = pose1.position.y
        start_pose.position.z = pose1.position.z
        start_pose.orientation = pose1.orientation
        waypoints.append(start_pose)

        # 目标姿态
        target_pose = Pose()
        target_pose.position.x = pose2.position.x
        target_pose.position.y = pose2.position.y
        target_pose.position.z = pose2.position.z
        target_pose.orientation = pose2.orientation
        waypoints.append(target_pose)

        # 规划笛卡尔路径
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # 执行路径
        if fraction > 0.9:
            rospy.loginfo("Executing Cartesian path.")
            self.group.execute(plan, wait=True)
        else:
            rospy.logerr("Failed to compute a valid Cartesian path.")

if __name__ == "__main__":
    try:
        # 初始化控制器
        controller = UR5MotionController()

        # 定义起始和目标关节位置（单位：弧度）
        point1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point2 = [0.5, -0.5, 1.0, -1.0, 0.5, -0.5]

        # 设置速度和加速度
        joints_velocity = 0.5  # 弧度/秒
        joints_acceleration = 0.5  # 弧度/秒²

        # 执行从point1到point2的关节空间运动
        controller.move_joint_space(point1, point2, joints_velocity, joints_acceleration)

        # 定义起始和目标姿态（Pose）
        pose1 = Pose()
        pose1.position.x = 0.3
        pose1.position.y = 0.2
        pose1.position.z = 0.5
        pose2 = Pose()
        pose2.position.x = 0.5
        pose2.position.y = -0.2
        pose2.position.z = 0.5

        # 设置线速度和加速度
        linear_velocity = 0.1  # 米/秒
        linear_acceleration = 0.1  # 米/秒²

        # 执行从pose1到pose2的笛卡尔空间运动
        controller.move_cartesian_space(pose1, pose2, linear_velocity, linear_acceleration)

        rospy.loginfo("Motion commands have been sent.")
    except rospy.ROSInterruptException:
        rospy.loginfo("UR5 Motion Controller Node Terminated.")
        pass
