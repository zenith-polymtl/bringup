from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ld = LaunchDescription()

    rgb_yolo = Node(
        package="vision",
        executable="rgb_yolo",
        name="rgb_yolo",
        parameters=[{
            'topic_name': "/zed/zed_node/rgb/color/rect/image",
        }],
    ) 

    ld.add_action(rgb_yolo)

    return ld