from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    water_web_server = Node(
        package = 'water_web_server_node',
        executable = 'water_web_server_node',
        name = 'water_web_server'
    )
    
    water_image_uploader = Node(
        package = 'upload_controller',
        executable = 'upload_controller',
        name = 'upload_image'
    )
    
    gcs_heartbeat = Node(
        package = 'tools',
        executable = 'gcs_heartbeat',
        name = 'gcs_heartbeat'
    )

    ld.add_action(water_web_server)
    ld.add_action(water_image_uploader)
    ld.add_action(gcs_heartbeat)

    return ld