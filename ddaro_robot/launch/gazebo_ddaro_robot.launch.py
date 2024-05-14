#!/usr/bin/env python3

from email.policy import default
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_file_name = 'ddaro.urdf'

    urdf = os.path.join(
        get_package_share_directory('ddaro_robot'), # package이름
        'description',
        'urdf',
        urdf_file_name) # urdf 파일 이름
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('ddaro_robot'),
        'rviz',
        'gazebo_ddaro.rviz')
    
    world = os.path.join(
        get_package_share_directory('ddaro_robot'), 
        'worlds', 
        'ddaro_human.world')
    
    os.environ["GAZEBO_MODEL_PATH"]="$GAZEBO_MODEL_PATH:/home/hyun/ros2_ws/src/ddaro"
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    rsp_params = {'robot_description': robot_desc}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  
    x_pose = LaunchConfiguration('x_pose', default='1.0')
    y_pose = LaunchConfiguration('y_pose', default='-4.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='1.0',
        description = 'Specify namespace of the robot'
    )
    
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='-4.0',
        description = 'Specify namespace of the robot'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world':world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[rsp_params, {'use_sim_time': use_sim_time}],
        arguments=[urdf]
    )
    
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[rsp_params, {'use_sim_time': use_sim_time}],
    )

    spawn_model_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity','ddaro_robot',
                   '-file', urdf,
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', '0.0',
                   '-package_to_model',
                   '-Y', '1.5708',
                   ],
                   output='screen'
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(spawn_model_cmd)
    ld.add_action(rviz_cmd)

    return ld