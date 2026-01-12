from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf', 'full_robot.urdf.xacro'
    )

    srdf_file = os.path.join(
        get_package_share_directory('robot_moveit'),
        'config', 'full_robot.srdf'
    )

    return LaunchDescription([
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': urdf_file}]
        # ),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {'robot_description': urdf_file},
                {'robot_description_semantic': srdf_file}
            ]
        )
    ])
