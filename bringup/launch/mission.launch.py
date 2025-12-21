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

    UI = Node(
        package = 'UI',
        executable = 'UI',
        name = 'UI'
    )


    ld.add_action(UI)
    ld.add_action(init)
    ld.add_action(convert)
    
    return ld