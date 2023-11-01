import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
# waffle pi

def generate_launch_description():
    
    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    walker = Node(
        package='walker',
        executable='walker'
    )
    
   

    return LaunchDescription([
        walker
    ])