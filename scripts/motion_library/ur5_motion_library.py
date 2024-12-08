import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb
from spatialmath import SE3
from std_msgs.msg import Header

class UR5MotionLibrary:
    def __init__(self):
        self.pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.sub = rospy.Subscriber('/joint_states', JointState, self._state_callback)
        self.robot = rtb.models.UR5()
        self.current_joint_state = None
        rospy.sleep(2)
        rospy.loginfo("UR5 Motion Library Initialized.")

    def _state_callback(self, msg):
        """Callback to update the current joint state."""
        self.current_joint_state = msg

    def read_robot_state(self):
        """Read the current robot joint positions."""
        if self.current_joint_state is None:
            rospy.logwarn("Robot state not yet available.")
            return None
        joint_positions = self.current_joint_state.position
        # rospy.loginfo(f"Current joint positions: {joint_positions}")
        return joint_positions

    def move_joint_space(self, point1, point2, velocity=1.0, acceleration=1.0):
        """Move the robot in joint space."""
        joint_traj = JointTrajectory()
        joint_traj.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        joint_durations = [abs(p2 - p1) / velocity for p1, p2 in zip(point1, point2)]
        max_duration = max(joint_durations)

        start_point = JointTrajectoryPoint()
        start_point.positions = point1
        start_point.velocities = [0.0] * 6
        start_point.accelerations = [0.0] * 6
        start_point.time_from_start = rospy.Duration(0.5)

        end_point = JointTrajectoryPoint()
        end_point.positions = point2
        end_point.velocities = [velocity] * 6
        end_point.accelerations = [acceleration] * 6
        end_point.time_from_start = rospy.Duration(max_duration)

        joint_traj.points = [start_point, end_point]
        rospy.loginfo("Publishing joint trajectory: %s to %s", point1, point2)
        self.pub.publish(joint_traj)
        rospy.sleep(max_duration + 1.0)

    def move_cartesian_space(self, start_xyz, end_xyz, linear_velocity=0.1, linear_acceleration=0.1, steps=100):
        """Move the robot in Cartesian space."""
        start_pose = SE3(start_xyz[0], start_xyz[1], start_xyz[2])
        end_pose = SE3(end_xyz[0], end_xyz[1], end_xyz[2])
        
        cartesian_path = rtb.ctraj(start_pose, end_pose, steps)
        joint_trajectory = []
        q_init = np.zeros(len(self.robot.q))

        for T in cartesian_path:
            sol = self.robot.ikine_LM(T, q0=q_init)
            if sol.success:
                joint_trajectory.append(sol.q)
                q_init = sol.q
            else:
                rospy.logwarn("IK solution failed for pose: %s", T)
                return

        joint_traj_msg = JointTrajectory()
        joint_traj_msg.header = Header()
        joint_traj_msg.header.stamp = rospy.Time.now()
        joint_traj_msg.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        total_distance = sum(np.linalg.norm(cartesian_path[i].t - cartesian_path[i - 1].t)
                             for i in range(1, len(cartesian_path)))
        time_per_step = total_distance / (linear_velocity * steps)

        for i, q in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            point.positions = q
            point.velocities = [linear_velocity] * len(q)
            point.accelerations = [linear_acceleration] * len(q)
            point.time_from_start = rospy.Duration((i + 1) * time_per_step)
            joint_traj_msg.points.append(point)

        rospy.loginfo("Publishing Cartesian trajectory.")
        self.pub.publish(joint_traj_msg)


class UR5MotionAPI:
    def __init__(self):
        self.controller = UR5MotionLibrary()
        rospy.loginfo("API: Initialization done.")

    def move_to_joint_positions(self, point1, point2, velocity=0.25, acceleration=0.1):
        """API to move robot in joint space."""
        rospy.loginfo("API: Moving robot in joint space from %s to %s", point1, point2)
        self.controller.move_joint_space(point1, point2, velocity, acceleration)

    def move_to_cartesian_positions(self, start_xyz, end_xyz, linear_velocity=0.1, linear_acceleration=0.1):
        """API to move robot in Cartesian space."""
        rospy.loginfo("API: Moving robot in Cartesian space from %s to %s", start_xyz, end_xyz)
        self.controller.move_cartesian_space(start_xyz, end_xyz, linear_velocity, linear_acceleration)

    def get_robot_state(self):
        """API to read the current robot state."""
        rospy.loginfo("API: Fetching current robot state.")
        return self.controller.read_robot_state()
