from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    diff_drive_controller = Node(
        package='diff_drive_controller',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[{
            'wheelbase': 0.6
        }]
        )
     
    return LaunchDescription([
        diff_drive_controller
    ])