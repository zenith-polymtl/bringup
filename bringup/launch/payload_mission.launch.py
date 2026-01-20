from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    init = Node(
        package="nav_stack",
        executable="init",
        name="init",
    )
    
    convert = Node(
        package="nav_stack",
        executable="convert",
        name="convert",
    )
    
    control_nav = Node(
        package="control_nav",
        executable="control_nav",
        name="control_nav",
    )

    ld.add_action(init)
    ld.add_action(convert)
    ld.add_action(control_nav)
    
    return ld