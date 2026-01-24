import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution,Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_robot_description, 'urdf', 'full_robot.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])
    empty_world = os.path.join(pkg_robot_description,'worlds',"empty.sdf")
    set_model = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=PathJoinSubstitution([
            FindPackageShare('robot_description'),'models'
        ])
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
                        'world':empty_world,
                        'verbose':'true',
                        }.items(),
    )   

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_robot_description, 'rviz', 'mycobot.rviz')]
    )
    joint_state_gui=Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller_position', '--controller-manager', '/controller_manager'],
    )#gripper_controller

    arm_timer = TimerAction(
        period=2.0,
        actions=[arm_controller_spawner]
    )
    joint_timer = TimerAction(
        period=0.0,
        actions=[joint_state_broadcaster_spawner]
    )
    gripper_timer = TimerAction(
        period=4.0,
        actions=[gripper_controller_spawner])

    ld = LaunchDescription()


    ld.add_action(set_model)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(gazebo)
    # ld.add_action(spawn_node)
    # ld.add_action(joint_timer)
    # ld.add_action(arm_timer)
    # ld.add_action(gripper_timer)
    ld.add_action(rviz_node)
    ld.add_action(joint_state_gui)

    return ld