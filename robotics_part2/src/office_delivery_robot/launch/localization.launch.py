import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('office_delivery_robot')
    default_map = os.path.join(pkg_share, 'maps', 'office_map_run2.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml_file,
        }],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # Shift map -> odom by +1.0m in X because office_map_run2 started at spawn S2 (-1.0, 0.0)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['1.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    position_tracker = Node(
        package='office_delivery_robot',
        executable='position_tracker_node',
        name='position_tracker_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('rviz', default_value='true'),
        map_server,
        lifecycle_mgr,
        map_to_odom_tf,
        position_tracker,
        rviz,
    ])
