#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'ns' argument for namespace
        DeclareLaunchArgument('ns', default_value='/', description='Namespace'),

        # Declare the 'px4_indoor_navigation' argument to specify the config path
        DeclareLaunchArgument('px4_indoor_navigation', default_value='/home/habib/invdro_ws/src/px4_indoor_navigation', description='Path to px4_indoor_navigation package'),

        # Set the namespace for the node
        PushRosNamespace(LaunchConfiguration('ns')),

        # Load parameters from a file
        Node(
            package='px4_indoor_navigation',
            executable='altitude_vxy_controller',  # Correct executable name
            name='altitude_vxy_controller',
            output='screen',
            parameters=[{
                'alt_vxy_params': PathJoinSubstitution([LaunchConfiguration('px4_indoor_navigation'), 'config', 'alt_vxy_params.yaml'])
            }],
            remappings=[],  # If needed, add remapping
            emulate_tty=True,  # Useful for ROS 2 logging
        ),
    ])
