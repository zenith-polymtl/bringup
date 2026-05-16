from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ld = LaunchDescription()

    tensor_yolo = Node(
        package="vision",
        executable="tensorRt_yolo",
        name="tensor_yolo",
        parameters=[{
            'topic_name': "/zed/zed_node/rgb/color/rect/image",
        }],
    ) 

    fps_test = Node(
        package="vision",
        executable="fps_counter",
        name="fps_counter"
    ) 


    ld.add_action(tensor_yolo)
    ld.add_action(fps_test)

    return ld