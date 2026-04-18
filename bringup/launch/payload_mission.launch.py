from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    init = Node(
        package="nav_stack",
        executable="init",
        name="init",
        parameters=[
            {"static_tf.parent_frame_name": "map",
            "static_tf.child_frame_name": "zed_camera_link",
            "static_tf.translation": [0.084, 0.0, -0.126], # A mesure : Camera est 3 cm en haut (XYZ = FLU)
            "static_tf.rotation": [0.0, -0.382683, 0.0, 0.923880]} #depend du 0 gimbal, jassume que identique
        ]
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
    
    mission_stats_controller = Node(
        package="mission_stats_controller",
        executable="mission_stats_controller",
        name="mission_stats_controller",
    )
    
    drone_heatbeat = Node(
        package="tools",
        executable="drone_heartbeat",
        name="drone_heartbeat",
        parameters=[{
            'topic_name': "/aeac/external/drone_heartbeat",
            'heartbeat_rate': 1.0,
        }],
    )    
    servo_controller = Node(
        package="payload",
        executable="payload",
        name="PayloadController",
    )
    rc_controller = Node(
        package="remote_controller_interface",
        executable="remote_controller_interface",
        name="remote_control_interface",
    )

    # ld.add_action(init)
    # ld.add_action(convert)
    # ld.add_action(control_nav)
    # ld.add_action(mission_stats_controller)
    ld.add_action(drone_heatbeat)
    ld.add_action(servo_controller)
    ld.add_action(rc_controller)
    
    return ld