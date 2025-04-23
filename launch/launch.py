import os
import launch
import launch_ros.actions

def generate_launch_desc():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_project_sc222ah',
            executable='first_step',
            name='colour_identifier',
            output='screen'
        )
    ])