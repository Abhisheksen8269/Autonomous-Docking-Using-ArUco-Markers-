from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch = os.path.join(pkg_tb3, 'launch', 'turtlebot3_world.launch.py')

    # Node to run aruco tracker
    aruco_node = Node(
        package='my_aruco_tracker',
        executable='aruco_tracker',
        name='aruco_tracker',
        output='screen',
        parameters=[{'marker_length': 0.10}]  # set your printed marker size in meters
    )

    # Optionally include the TB3 gazebo launch (if you have it)
    include_tb3 = IncludeLaunchDescription(PythonLaunchDescriptionSource(tb3_launch))

    return LaunchDescription([
        include_tb3,
        aruco_node,
    ])
