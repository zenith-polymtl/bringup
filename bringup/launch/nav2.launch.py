from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = "/absolute/path/to/controller.yaml"

    return LaunchDescription([
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[params_file],
        ),
    ])