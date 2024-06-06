from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the Realsense launch file
    realsense_launch_file_dir = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch'
    )
    realsense_launch_file = os.path.join(realsense_launch_file_dir, 'rs_launch.py')
    
    # Include the Realsense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file)
    )
    
    # Define the ddaro_gui node
    ddaro_gui_node = Node(
        package='ddaro_gui',
        executable='ddaro_gui',
        name='ddaro_gui_node',
        output='log'
    )

    # Create and return the launch description
    return LaunchDescription([
        realsense_launch,
        ddaro_gui_node
    ])

