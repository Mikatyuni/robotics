"""Start the Nav2 navigation stack with the A* or Dijkstra planner config.

Run AFTER the simulation and localization are up:
    Terminal 1: ros2 launch office_delivery_robot office_world.launch.py
    Terminal 2: ros2 launch office_delivery_robot localization.launch.py
    Terminal 3: ros2 launch office_delivery_robot navigation.launch.py planner:=astar

Nav2 publishes Twist on /cmd_vel, which ros_gz_bridge forwards to the Gazebo
DiffDrive plugin (see config/office_bridge.yaml). Do not run office_drive_node
at the same time: it also publishes on /cmd_vel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_nav2(context):
    pkg_share = get_package_share_directory('office_delivery_robot')
    planner = LaunchConfiguration('planner').perform(context)
    if planner not in ('astar', 'dijkstra'):
        raise RuntimeError(f"planner must be 'astar' or 'dijkstra', got '{planner}'")
    params = os.path.join(pkg_share, 'config', f'nav2_{planner}.yaml')
    nav2_share = get_package_share_directory('nav2_bringup')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='astar',
                              description="Global planner config: 'astar' or 'dijkstra'"),
        OpaqueFunction(function=launch_nav2),
    ])
