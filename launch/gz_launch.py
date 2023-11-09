import sys
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument('description', default_value='false',
        description='Launch turtlebot4 description'
    ),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    rosbag_record = sys.argv[-1]
    
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    
    # Bring up World
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )
    
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )
    
    walker = Node(
        package='walker',
        executable='walker',
        output='screen',
    )
    
    
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    ld.add_action(walker)
    if rosbag_record=='record:=True':
        ld.add_action(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", "walker_bag", "-a"]
            )
        )
    return ld