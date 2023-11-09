import sys
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


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
    # pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    # pkg_turtlebot4_viz = get_package_share_directory('turtlebot4_viz')
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')

    
    # description_launch = PathJoinSubstitution(
    #     [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py']
    # )
    
    # Bring up World
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    
    # ignition_launch = PathJoinSubstitution(
    #     [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_ignition.launch.py'])
    
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
    
    # xacro_file = PathJoinSubstitution([pkg_turtlebot4_description,
    #                                    'urdf',
    #                                    LaunchConfiguration('model'),
    #                                    'turtlebot4.urdf.xacro'])
    
    # namespace = LaunchConfiguration('namespace')
    
    # rviz2_config = PathJoinSubstitution(
    #     [pkg_turtlebot4_viz, 'rviz', 'model.rviz'])
    
    # rviz = GroupAction([
    #     PushRosNamespace(namespace),

    #     Node(package='rviz2',
    #          executable='rviz2',
    #          name='rviz2',
    #          arguments=['-d', rviz2_config],
    #          parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #          remappings=[
    #             ('/tf', 'tf'),
    #             ('/tf_static', 'tf_static')
    #          ],
    #          output='screen'),

    #     # Delay launch of robot description to allow Rviz2 to load first.
    #     # Prevents visual bugs in the model.
    #     TimerAction(
    #         period=3.0,
    #         actions=[
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource([description_launch]),
    #                 launch_arguments=[('model', LaunchConfiguration('model'))],
    #                 condition=IfCondition(LaunchConfiguration('description'))
    #             )])
    # ])
    
    

    
    
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
    #         {'robot_description': Command([
    #             'xacro', ' ', xacro_file, ' ',
    #             'gazebo:=ignition', ' ',
    #             'namespace:=', namespace])},
    #     ],
    #     remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static')
    #     ]
    # )
    
    
    
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static')
    #     ]
    # )
    
    
    walker = Node(
        package='walker',
        executable='walker'
    )
    
   

    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    # ld.add_action(map_arg)
    # ld.add_action(localization_params_arg)
    # ld.add_action(localization)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    # ld.add_action(robot_state_publisher)
    
    # ld.add_action(rviz)
    # ld.add_action(joint_state_publisher)
    ld.add_action(walker)
    return ld