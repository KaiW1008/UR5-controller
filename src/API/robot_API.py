from motion_library.ur5_motion_library import UR5MotionLibrary
import rospy

class UR5MotionAPI:
    def __init__(self):
        self.controller = UR5MotionLibrary()

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
