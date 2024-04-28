from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ddaro_gui_node = Node(
        package='ddaro_gui',
        executable='ddaro_gui',
        name='ddaro_gui_node',
        output='log'
    )

    return LaunchDescription([
        ddaro_gui_node
    ])