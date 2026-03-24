#!/usr/bin/env python3

import os
import re
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Setup launch configuration."""

    start_serial = LaunchConfiguration("start_serial")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    serial_input_unit = LaunchConfiguration("serial_input_unit")
    serial_control_rate_hz = LaunchConfiguration("serial_control_rate_hz")
    serial_max_speed_deg_s = LaunchConfiguration("serial_max_speed_deg_s")
    serial_target_alpha = LaunchConfiguration("serial_target_alpha")
    serial_min_deg = LaunchConfiguration("serial_min_deg")
    serial_max_deg = LaunchConfiguration("serial_max_deg")
    serial_publish_deadband_deg = LaunchConfiguration("serial_publish_deadband_deg")

    # Get the package share directory
    pkg_share = FindPackageShare('my_servo_sim').find('my_servo_sim')

    # Resolve xacro file
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_servo.xacro')

    # Generate URDF from xacro in Python to avoid parser issues with command-line param overrides.
    robot_description_xml = xacro.process_file(urdf_file).toxml()
    robot_description_xml = re.sub(r"^\s*<\?xml[^>]*\?>\s*", "", robot_description_xml)
    robot_description_xml = re.sub(r"<!--.*?-->", "", robot_description_xml, flags=re.DOTALL)
    robot_description = {
        'robot_description': robot_description_xml
    }

    # Node: Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        name='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Node: Gazebo
    gazebo_node = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        shell=False,
    )

    # Node: Spawn Robot Entity
    spawn_entity_node = Node(
        package='gazebo_ros',
        name='urdf_spawner',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'my_servo',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
    )

    # Spawn controllers in sequence to avoid activation race conditions.
    joint_state_spawner = Node(
        package='controller_manager',
        name='joint_state_spawner',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
            '--activate-as-group',
        ],
        output='screen',
    )

    servo_spawner = Node(
        package='controller_manager',
        name='servo_spawner',
        executable='spawner',
        arguments=[
            'servo_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
            '--activate-as-group',
        ],
        output='screen',
    )

    start_joint_state_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_spawner],
        )
    )

    start_servo_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[servo_spawner],
        )
    )

    # Optional serial bridge so one launch command starts everything.
    serial_bridge_node = Node(
        package='my_servo_sim',
        executable='serial_to_servo.py',
        name='serial_to_servo',
        output='screen',
        condition=IfCondition(start_serial),
        parameters=[{
            'port': serial_port,
            'baud': serial_baud,
            'input_unit': serial_input_unit,
            'control_rate_hz': serial_control_rate_hz,
            'max_speed_deg_s': serial_max_speed_deg_s,
            'target_alpha': serial_target_alpha,
            'min_deg': serial_min_deg,
            'max_deg': serial_max_deg,
            'publish_deadband_deg': serial_publish_deadband_deg,
        }],
    )

    return [
        robot_state_publisher_node,
        gazebo_node,
        spawn_entity_node,
        start_joint_state_after_spawn,
        start_servo_after_joint_state,
        serial_bridge_node,
    ]


def generate_launch_description():
    """Generate launch description."""

    return LaunchDescription(
        [
            DeclareLaunchArgument('start_serial', default_value='true'),
            DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
            DeclareLaunchArgument('serial_baud', default_value='115200'),
            DeclareLaunchArgument('serial_input_unit', default_value='deg'),
            DeclareLaunchArgument('serial_control_rate_hz', default_value='60.0'),
            DeclareLaunchArgument('serial_max_speed_deg_s', default_value='45.0'),
            DeclareLaunchArgument('serial_target_alpha', default_value='0.2'),
            DeclareLaunchArgument('serial_min_deg', default_value='-90.0'),
            DeclareLaunchArgument('serial_max_deg', default_value='90.0'),
            DeclareLaunchArgument('serial_publish_deadband_deg', default_value='0.1'),
            OpaqueFunction(function=launch_setup),
        ]
    )
