import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    xacro_file = os.path.join(get_package_share_directory("diff_drive_robot"), "urdf/diff_drive_robot.xacro")   
    robot_description_config = xacro.process_file(xacro_file, mappings={})
    robot_description = robot_description_config.toxml()
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py']),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
      )
    
    params = {'use_sim_time': True, 'robot_description': robot_description}
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])
    
    spawn_robot = Node(package='ros_gz_sim', executable='create',
            arguments=[
                '-name', "diff_drive_robot",
                '-x', '0',
                '-z', '0.6',
                '-Y', '0',
                '-topic', '/robot_description'],
            output='screen')
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        output='screen',
        arguments=['joint_state_broadcaster']
        )
    
    gazebo_wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='wheel_controller',
        output='screen',
        arguments=['wheel_controller']
        )
    
    dd_param_file = os.path.join(get_package_share_directory("dd_controller"), "config/dd_controller.yaml")
    dd_controller = Node(
        package='dd_controller',
        executable='dd_controller',
        name='dd_controller',
        output='screen',
        parameters=[dd_param_file],
        remappings=[
            ('/wheel_velocities', '/wheel_controller/commands'),
        ],
        ) 
     
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        gazebo_wheel_controller,
        dd_controller
    ])