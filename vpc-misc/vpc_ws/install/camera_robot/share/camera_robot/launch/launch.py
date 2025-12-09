from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('camera_robot')
    world_file = os.path.join(pkg_share, 'worlds', 'world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'my_robot',
                       '-file', urdf_file],
            output='screen'),
    ])
