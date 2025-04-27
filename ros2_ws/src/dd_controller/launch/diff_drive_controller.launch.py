from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    dd_controller = Node(
        package='dd_controller',
        executable='dd_controller',
        name='dd_controller',
        output='screen',
        parameters=[{
            'wheel_base': 0.4,
            'wheel_radius': 0.6,
            'left_wheels': ['front_left_wheel', 'back_left_wheel'],
            'right_wheels': ['front_right_wheel', 'back_right_wheel']
        }]
        ) 
    
    return LaunchDescription([
        dd_controller
    ])
