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
            'yolo_model': "models/best-medium.pt",
        }],
    ) 

    fps_test = Node(
        package="vision",
        executable="fps_counter",
        name="fps_counter"
    ) 


    ld.add_action(rgb_yolo)
    ld.add_action(fps_test)

    return ld