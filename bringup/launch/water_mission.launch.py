from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ld = LaunchDescription()

    init = Node(
        package="nav_stack",
        executable="init",
        name="init",
        parameters=[
            {"static_tf.parent_frame_name": "gimbal_link",
            "static_tf.child_frame_name": "zed_link",
            "static_tf.translation": [0.0, 0.0, 0.03], # A mesure : Camera est 3 cm en haut (XYZ = FLU)
            "static_tf.rotation": [0.0, 0.0, 0.0, 1.0]} #depend du 0 gimbal, jassume que identique
        ]
    )
    '''
    self.declare_parameter("static_tf.parent_frame_name", "gimbal_link")
    self.declare_parameter("static_tf.child_frame_name", "camera_link")
    self.declare_parameter("static_tf.translation", [0.084, 0.0, -0.126])
    self.declare_parameter("static_tf.rotation", [0.0, -0.382683, 0.0, 0.923880])
    "static_tf.parent_frame_name": "base_link",
    "static_tf.child_frame_name": "zed_camera_link",
    "static_tf.translation": [0.0, 0.0, 0.03], # A mesure : Camera est 3 cm en haut (XYZ = FLU)
    "static_tf.rotation": [0.0, 0.0, 0.0, 1.0]
    '''    
    convert = Node(
        package="nav_stack",
        executable="convert",
        name="convert",
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    control_nav = Node(
        package="control_nav",
        executable="control_nav",
        name="control_nav",
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    mission_stats_controller = Node(
        package="state_w",
        executable="auto_approach",
        name="autonomous_approach",
        parameters=[{
            #'use_sim_time': True,
            'sim': True,
        }]
    )
    
    target_mock = Node(
        package="state_w",
        executable="target_detection_mock",
        name="target_detection_mock",
        parameters=[{
            'use_sim_time': True,
        }]
    )
    
    gimbal_controller = Node(
        package="gimbal_controller",
        executable="gimbal_mavros",
        name="gremsy_mavros_ctrl",
        parameters=[{
            'use_sim_time': False,
        }]
    )

    
    image_capture = Node(
        package="shoot_and_capture",
        executable="image_capture",
        name="image_capture",
        parameters=[{
            'image_topic' : "/zed/zed_node/rgb/color/rect/image/compressed",
            'save_dir' : '/aeac/workspaces/water_ws/snapshots'
        }]
    )
    
    image_transfer = Node(
        package="shoot_and_capture",
        executable="image_transfer",
        name="image_transfer"
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

    valve = Node(
        package="water_payload",
        executable="valve_control",
        name="valve",
        parameters=[{
            'burst_time': 3.0
        }],
    ) 
    
    position_node = Node(
            package="polar_system",
            executable="polar",
            name="polar",
            parameters=[{
                # Topics / frame
                'topic_pose': "/mavros/local_position/pose",
                'topic_vel': "/mavros/local_position/velocity_local",
                'topic_goal_polar': "/aeac/internal/auto_approach/target_position",
                'topic_estimated_center': "/aeac/internal/auto_shoot/center_location",
                'topic_activation': "/aeac/internal/auto_approach/activate_polar",
                'topic_ctrl_activation': "/controller_activation",
                'topic_raw_setpoint': "/mavros/setpoint_raw/local",
                'topic_reached_target': "/aeac/internal/auto_approach/in_position",
                'topic_abort' : "/aeac/external/mission/abort_all",

                'frame_id': "map",

                # Rates / filters
                'alpha': 0.0, #Higher means more rate v_r rate allowed

                # Limits
                'centripetal_limit': 1.5,
                'minimal_margin': 2.0,
                'soft_repulsion_initial_radius': 5.0,

                # CSV log
                'csv_path': "approach_log_polar.csv",

                # MAVLink config
                'set_msg_interval': True,
                'msg_interval_rate': 25.0,

                # verbose
                'talk': True,
                'log': True,

                # Unified PID params (kp, ki, kd, max_i, max_out, deriv_tau, d_clip)
                # pid_r
                'pid_r_kp': 2.2, 'pid_r_ki': 1.0, 'pid_r_kd': 0.2, 'pid_r_max_i': 0.2, 'pid_r_max_out': 7.0, 'pid_r_deriv_tau': 0.0, 'pid_r_d_clip': 1.0,
                # pid_r_abs
                'pid_r_abs_kp': 0.2, 'pid_r_abs_ki': 0.1, 'pid_r_abs_kd': 0.1, 'pid_r_abs_max_i': 0.3, 'pid_r_abs_max_out': 2.0, 'pid_r_abs_deriv_tau': 0.1, 'pid_r_abs_d_clip': 0.0,
                # pid_r_hold
                'pid_r_hold_kp': 0.2, 'pid_r_hold_ki': 0.1, 'pid_r_hold_kd': 0.12, 'pid_r_hold_max_i': 0.5, 'pid_r_hold_max_out': 3.0, 'pid_r_hold_deriv_tau': 0.1, 'pid_r_hold_d_clip': 0.0,
                # pid_theta_abs
                'pid_theta_abs_kp': 0.3, 'pid_theta_abs_ki': 0.1, 'pid_theta_abs_kd': 0.15, 'pid_theta_abs_max_i': 0.5, 'pid_theta_abs_max_out': 3.0, 'pid_theta_abs_deriv_tau': 0.1, 'pid_theta_abs_d_clip': 0.0,
                # pid_theta_hold
                'pid_theta_hold_kp': 0.6, 'pid_theta_hold_ki': 0.2, 'pid_theta_hold_kd': 0.3, 'pid_theta_hold_max_i': 0.5, 'pid_theta_hold_max_out': 3.0, 'pid_theta_hold_deriv_tau': 0.1, 'pid_theta_hold_d_clip': 0.0,
                # pid_v_theta
                'pid_v_theta_kp': 2.2, 'pid_v_theta_ki': 1.0, 'pid_v_theta_kd': 0.0, 'pid_v_theta_max_i': 0.2, 'pid_v_theta_max_out': 2.25, 'pid_v_theta_deriv_tau': 0.0, 'pid_v_theta_d_clip': 0.3,
                # pid_z
                'pid_z_kp': 0.6, 'pid_z_ki': 0.0, 'pid_z_kd': 0.3, 'pid_z_max_i': 1.0, 'pid_z_max_out': 3.0, 'pid_z_deriv_tau': 0.075, 'pid_z_d_clip': 0.0,
                # pid_z_abs
                'pid_z_abs_kp': 0.3, 'pid_z_abs_ki': 0.0, 'pid_z_abs_kd': 0.25, 'pid_z_abs_max_i': 0.5, 'pid_z_abs_max_out': 3.0, 'pid_z_abs_deriv_tau': 0.1, 'pid_z_abs_d_clip': 0.8,
                # pid_z_hold
                'pid_z_hold_kp': 0.3, 'pid_z_hold_ki': 0.1, 'pid_z_hold_kd': 0.15, 'pid_z_hold_max_i': 0.5, 'pid_z_hold_max_out': 3.0, 'pid_z_hold_deriv_tau': 0.1, 'pid_z_hold_d_clip': 0.0,
                # pid_yaw
                'pid_yaw_kp': 1.5, 'pid_yaw_ki': 1.0, 'pid_yaw_kd': 0.24, 'pid_yaw_max_i': 0.4, 'pid_yaw_max_out': 6.0, 'pid_yaw_deriv_tau': 0.0, 'pid_yaw_d_clip': 0.05,
            }]
        )

    fake_front_target = Node(
        package="polar_system",
        executable="fake_front_target",
        name="fake_front_target",
        parameters=[{
            'topic_pose': "/mavros/local_position/pose",
            'topic_heading': "/mavros/global_position/compass_hdg",
            'topic_activation': "/aeac/internal/auto_approach/activate_polar",
            'topic_front_point': "/aeac/internal/auto_shoot/center_location",
            'front_point_distance': 5.0,
        }]
    )

    servo_node = Node(
        package="water_payload",
        executable="servo_control",
        name="servo_controller",
    )

    rc_interface = Node(
        package="remote_controller_interface",
        executable="remote_controller_interface",
        name="rc_interface",
    )
    
    ld.add_action(valve)
    ld.add_action(servo_node)
    ld.add_action(init)
    ld.add_action(convert)
    ld.add_action(control_nav)
    # ld.add_action(gimbal_test_node)
    ld.add_action(gimbal_controller)
    ld.add_action(mission_stats_controller)
    ld.add_action(image_capture)
    ld.add_action(drone_heatbeat)
    ld.add_action(image_transfer)
    ld.add_action(rc_interface)
    ld.add_action(position_node)
    ld.add_action(fake_front_target)

    return ld