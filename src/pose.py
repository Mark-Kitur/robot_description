# #!/usr/bin/env python3

# import sys
# import rclpy
# import threading
# from rclpy.node import Node
# from geometry_msgs.msg import Pose, PoseStamped
# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
# import numpy as np
# import math

# # MoveIt Python API imports
# from moveit.core.robot_state import RobotState
# from moveit.core.planning_scene import PlanningScene
# from moveit.planning import (
#     MoveItPy,
#     MultiPipelinePlanRequestParameters,
# )
# from moveit.core.kinematic_constraints import (
#     construct_joint_constraint,
#     construct_goal_constraints,
#     KinematicConstraint
# )

# class PandaMoveToPoint:
#     def __init__(self, node):
#         self.node = node
#         self.node.get_logger().info("Initializing MoveItPy...")
        
#         # Initialize MoveItPy instance
#         # The robot name should match your move group configuration
#         self.moveit = MoveItPy(node_name="moveit_py_node")
        
#         # Get the planning component for panda_arm
#         self.panda_arm = self.moveit.get_planning_component("panda_arm")
        
#         self.node.get_logger().info("MoveItPy initialized successfully!")
    
#     def goto(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
#         """Move end effector to specified pose"""
        
#         # Create target pose
#         target_pose = Pose()
#         target_pose.position.x = float(x)
#         target_pose.position.y = float(y)
#         target_pose.position.z = float(z)
        
#         # Convert RPY to quaternion
#         q = self.rpy_to_quaternion(roll, pitch, yaw)
#         target_pose.orientation.x = q[0]
#         target_pose.orientation.y = q[1]
#         target_pose.orientation.z = q[2]
#         target_pose.orientation.w = q[3]
        
#         self.node.get_logger().info(f"Moving to pose: position=({x}, {y}, {z})")
        
#         # Set the goal pose
#         self.panda_arm.set_goal_state(pose_stamped_msg=self.create_pose_stamped(target_pose))
        
#         # Plan to the goal
#         plan_result = self.panda_arm.plan()
        
#         if not plan_result:
#             self.node.get_logger().error("Planning failed!")
#             return False
        
#         # Execute the plan
#         self.node.get_logger().info("Executing plan...")
#         self.moveit.execute(plan_result)
        
#         return True
    
#     def goto_joints(self, joint_positions):
#         """Move to specific joint positions"""
#         if len(joint_positions) != 7:
#             self.node.get_logger().error("Panda arm needs 7 joint values")
#             return False
        
#         # Set joint goal
#         self.panda_arm.set_goal_state(joint_positions=joint_positions)
        
#         # Plan and execute
#         plan_result = self.panda_arm.plan()
        
#         if not plan_result:
#             self.node.get_logger().error("Planning failed!")
#             return False
        
#         self.node.get_logger().info("Executing joint space plan...")
#         self.moveit.execute(plan_result)
        
#         return True
    
#     def create_pose_stamped(self, pose, frame_id="world"):
#         """Create a PoseStamped message"""
#         pose_stamped = PoseStamped()
#         pose_stamped.header.frame_id = frame_id
#         pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
#         pose_stamped.pose = pose
#         return pose_stamped
    
#     def rpy_to_quaternion(self, roll, pitch, yaw):
#         """Convert roll, pitch, yaw (in radians) to quaternion"""
#         cy = math.cos(yaw * 0.5)
#         sy = math.sin(yaw * 0.5)
#         cp = math.cos(pitch * 0.5)
#         sp = math.sin(pitch * 0.5)
#         cr = math.cos(roll * 0.5)
#         sr = math.sin(roll * 0.5)
        
#         qw = cr * cp * cy + sr * sp * sy
#         qx = sr * cp * cy - cr * sp * sy
#         qy = cr * sp * cy + sr * cp * sy
#         qz = cr * cp * sy - sr * sp * cy
        
#         return [qx, qy, qz, qw]
    
#     def get_current_pose(self):
#         """Get current end effector pose"""
#         robot_state = self.moveit.get_robot_state()
#         # Panda hand link name might be "panda_hand" or "panda_link8"
#         # Check your URDF for the correct end effector link name
#         if robot_state is not None:
#             # This is a simplified version - in reality you'd need to 
#             # get the transform from robot state
#             return robot_state
#         return None

# def main():
#     rclpy.init()
    
#     # Create node
#     node = Node("panda_move_to_point")
    
#     # Create the mover
#     mover = PandaMoveToPoint(node)
    
#     # Give some time for initialization
#     import time
#     time.sleep(2.0)
    
#     # Parse command line arguments
#     if len(sys.argv) < 4:
#         print("\n=== Panda Arm MoveToPoint Controller ===")
#         print("Usage: python3 panda_move_to_point.py x y z [roll] [pitch] [yaw]")
#         print("All values in meters and radians")
#         print("\nExamples:")
#         print("  python3 panda_move_to_point.py 0.4 0.0 0.6")
#         print("  python3 panda_move_to_point.py 0.4 0.0 0.6 3.14 0.0 0.0")
#         print("\nJoint space control:")
#         print("  python3 panda_move_to_point.py --joints 0.0 -0.785 0.0 -2.356 0.0 1.571 0.785")
#     else:
#         if sys.argv[1] == "--joints":
#             # Joint space control
#             if len(sys.argv) < 9:
#                 print("Need 7 joint values for --joints mode")
#                 return
#             joint_positions = [float(x) for x in sys.argv[2:9]]
#             print(f"Moving to joint positions: {joint_positions}")
#             mover.goto_joints(joint_positions)
#         else:
#             # Cartesian space control
#             x = float(sys.argv[1])
#             y = float(sys.argv[2])
#             z = float(sys.argv[3])
            
#             roll = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
#             pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
#             yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
            
#             print(f"Moving to pose:")
#             print(f"  Position: ({x}, {y}, {z}) meters")
#             print(f"  Orientation (RPY): ({roll}, {pitch}, {yaw}) radians")
            
#             success = mover.goto(x, y, z, roll, pitch, yaw)
            
#             if success:
#                 print("Movement completed successfully!")
#             else:
#                 print("Movement failed!")
    
#     # Keep the node alive to finish execution
#     time.sleep(2.0)
    
#     # Cleanup
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()