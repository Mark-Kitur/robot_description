#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


names_of_joints = [
    'link1_to_link2',
    'link2_to_link3',
    'link3_to_link4',
    'link4_to_link5',
    'link5_to_link6',
    'link6_to_flange_link'
]

class ControlRobot(Node):
    def __init__(self):
        super().__init__("basic_joint_trajectory_publisher")

        # Arm controller publisher
        self.pose_publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10
        )

        # Gripper action client
        self.gripper_action = ActionClient(
            self, GripperCommand,
            "/gripper_controller_position/gripper_cmd"
        )  

        self.get_logger().info("Waiting for gripper action server...")
        self.gripper_action.wait_for_server()
        self.get_logger().info("Gripper action server ready!")

        # Timer
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Precomputed positions
        self.positions=[
                1.0979,1.5236,1.5707, 1.5236, 0.0,0.0
            ]

        self.gripper_open = 0.12
        self.gripper_closed = -0.63

        self.index = 0
        self.forward = True

    def timer_callback(self):
        # Publish arm motion
        traj = JointTrajectory()
        traj.joint_names = names_of_joints

        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start = Duration(sec=1)

        traj.points.append(point)
        self.pose_publisher.publish(traj)

        # Alternate gripper command
        grip_target = (
            self.gripper_open if (self.index % 2 == 0)
            else self.gripper_closed
        )
        self.send_gripper_command(grip_target)

        # Update index
        if self.forward:
            if self.index < len(self.positions) - 1:
                self.index += 1
            else:
                self.forward = False
        else:
            if self.index > 0:
                self.index -= 1
            else:
                self.forward = True

    def send_gripper_command(self, position, effort=5.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)
 
        self.get_logger().info(f"Sending gripper command: {position}")

        future = self.gripper_action.send_goal_async(goal)
        future.add_done_callback(self.gripper_goal_response)

    def gripper_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected!")
            return

        self.get_logger().info("Gripper goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Gripper result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlRobot()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
