from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the gazebo launch file
    gazebo_launch_file_dir = os.path.join(
        get_package_share_directory('ddaro_robot'),
        'launch'
    )
    gazebo_launch_file = os.path.join(gazebo_launch_file_dir, 'gazebo_ddaro_robot.launch.py')
    
    # Include the gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )
    
    # Path to the nav2 launch file
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('ddaro_robot'),
        'launch'
    )
    nav2_launch_file = os.path.join(gazebo_launch_file_dir, 'navigation2.launch.py')
    
    # Include the nav2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file)
    )
    
     # Path to the core launch file
    core_launch_file_dir = os.path.join(
        get_package_share_directory('ddaro_core'),
        'launch'
    )
    core_launch_file = os.path.join(core_launch_file_dir, 'ddaro_core.launch.py')
    
    # Include the core launch file
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_launch_file)
    )
    
     # Path to the gui launch file
    gui_launch_file_dir = os.path.join(
        get_package_share_directory('ddaro_gui'),
        'launch'
    )
    gui_launch_file = os.path.join(gui_launch_file_dir, 'ddaro_gui.launch.py')
    
    # Include the gui launch file
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gui_launch_file)
    )

    # Create and return the launch description
    return LaunchDescription([
	    nav2_launch,
        gazebo_launch,
	    core_launch,
	    gui_launch
    ])

