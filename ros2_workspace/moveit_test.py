#!/home/alexander/venv/bin/python

import rclpy
from rclpy.node import Node

from moveit_commander.robot import RobotCommander
from moveit_commander.planning_scene import PlanningSceneInterface
from moveit_commander.move_group import MoveGroupCommander

def main():
    # Initialize ROS2
    rclpy.init()
    node = Node("moveit2_5dof_example")

    # Initialize MoveIt2 interfaces
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "arm"  # replace with your MoveIt2 group name
    move_group = MoveGroupCommander(group_name)

    node.get_logger().info("MoveIt2 initialized!")

    # Example 1: Move to joint positions
    target_joints = [0.0, 0.0, 2.618, 3.1416]  # base, shoulder, elbow, hand
    move_group.set_joint_value_target(target_joints)
    node.get_logger().info(f"Moving to joint positions: {target_joints}")
    move_group.go(wait=True)
    move_group.stop()  # ensure no residual movement

    # Example 2: Move to a Cartesian pose
    from geometry_msgs.msg import Pose
    target_pose = Pose()
    target_pose.position.x = 0.17   # meters
    target_pose.position.y = -0.0078
    target_pose.position.z = -0.054
    target_pose.orientation.w = 1.0  # no rotation

    move_group.set_pose_target(target_pose)
    node.get_logger().info(f"Moving to Cartesian pose: {target_pose}")
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    node.get_logger().info("Movement complete!")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
