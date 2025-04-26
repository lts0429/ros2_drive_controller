from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # dd_controller = Node(
    #     package='dd_controller',
    #     executable='dd_controller',
    #     name='dd_controller',
    #     output='screen',
    #     parameters=[{
    #         'wheelbase': 0.6
    #     }]
    #     )

    dd_controller = Node(
        package='dd_controller',
        executable='dd_controller',
        name='dd_controller',
        output='screen',
        parameters=['path/to/your/config.yaml']
        ) 
    
    return LaunchDescription([
        dd_controller
    ])
