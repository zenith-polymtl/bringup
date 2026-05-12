"""
vision_pipeline.launch.py — Pipeline complet AEAC

Nodes :
  1. system_transformation_node  (wall_detector)
  2. scene_descriptor_node       (scene_descriptor)
  3. overlay_node

Exemples :
  # Test rosbag (défauts)
  ros2 launch bringup vision_mission1_pipeline.launch.py

  # Hardware ZED 2i (bbox en résolution image 1920×1080)
  ros2 launch bringup vision_mission1_pipeline.launch.py \
      bbox_width:=1920 bbox_height:=1080 \
      objects_topic:=/zed/zed_node/obj_det/objects

  # Caméra inclinée, cap Est, debug
  ros2 launch bringup vision_mission1_pipeline.launch.py \
      camera_pitch_deg:=45.0 drone_heading_deg:=90.0 log_level:=debug
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ═══════════════════════════════════════════════════════════════════════
    # ARGUMENTS
    # ═══════════════════════════════════════════════════════════════════════
    args = [
        # ── Caméra / drone ───────────────────────────────────────────────
        DeclareLaunchArgument('camera_pitch_deg',  default_value='0.0',
            description='Inclinaison caméra vers le bas (degrés)'),
        DeclareLaunchArgument('drone_heading_deg', default_value='0.0',
            description='Cap drone (0=Nord, 90=Est, 180=Sud, 270=Ouest)'),

        # ── Résolution bbox ──────────────────────────────────────────────
        # Test rosbag  : bbox en coordonnées du cloud → 448×256
        # Hardware ZED : bbox en coordonnées image RGB → 1280×720 ou 1920×1080
        DeclareLaunchArgument('bbox_width',  default_value='448',
            description='Résolution de référence des bbox ZED (largeur)'),
        DeclareLaunchArgument('bbox_height', default_value='256',
            description='Résolution de référence des bbox ZED (hauteur)'),

        # ── RANSAC ───────────────────────────────────────────────────────
        DeclareLaunchArgument('ransac_dist',       default_value='0.03'),
        DeclareLaunchArgument('ransac_iterations', default_value='50'),
        DeclareLaunchArgument('min_inlier_ratio',  default_value='0.50'),
        DeclareLaunchArgument('padding_ratio',     default_value='0.20'),

        # ── Synchronisation ──────────────────────────────────────────────
        DeclareLaunchArgument('sync_tolerance',  default_value='8.00'),
        DeclareLaunchArgument('image_tolerance', default_value='10.00'),
        DeclareLaunchArgument('buffer_size',     default_value='2'),

        # ── Plans coplanaires ────────────────────────────────────────────
        DeclareLaunchArgument('plane_sim_thresh',  default_value='0.85'),
        DeclareLaunchArgument('plane_dist_thresh', default_value='0.35'),

        # ── Topics overlay (configurables pour test vs hardware) ─────────
        DeclareLaunchArgument('objects_topic',
            default_value='/aeac/test/objects',
            description='Topic objets ZED. '
                        'Test: /aeac/test/objects  '
                        'Hardware: /zed/zed_node/obj_det/objects'),
        DeclareLaunchArgument('scene_text_topic',
            default_value='/aeac/internal/scene_text',
            description='Topic texte scene_descriptor'),

        # ── Sauvegarde overlay ───────────────────────────────────────────
        DeclareLaunchArgument('save_images', default_value='true'),
        DeclareLaunchArgument('save_dir',    default_value='/tmp/aeac_overlay'),
        DeclareLaunchArgument('max_saved',   default_value='20'),

        # ── Logs ─────────────────────────────────────────────────────────
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    # ═══════════════════════════════════════════════════════════════════════
    # SUBSTITUTIONS
    # ═══════════════════════════════════════════════════════════════════════
    camera_pitch    = LaunchConfiguration('camera_pitch_deg')
    heading         = LaunchConfiguration('drone_heading_deg')
    bbox_w          = LaunchConfiguration('bbox_width')
    bbox_h          = LaunchConfiguration('bbox_height')
    ransac_dist     = LaunchConfiguration('ransac_dist')
    ransac_iter     = LaunchConfiguration('ransac_iterations')
    min_inliers     = LaunchConfiguration('min_inlier_ratio')
    padding         = LaunchConfiguration('padding_ratio')
    sync_tol        = LaunchConfiguration('sync_tolerance')
    img_tol         = LaunchConfiguration('image_tolerance')
    buf_size        = LaunchConfiguration('buffer_size')
    plane_sim       = LaunchConfiguration('plane_sim_thresh')
    plane_dist      = LaunchConfiguration('plane_dist_thresh')
    objects_topic   = LaunchConfiguration('objects_topic')
    scene_text_topic= LaunchConfiguration('scene_text_topic')
    save_images     = LaunchConfiguration('save_images')
    save_dir        = LaunchConfiguration('save_dir')
    max_saved       = LaunchConfiguration('max_saved')
    log_level       = LaunchConfiguration('log_level')

    # ═══════════════════════════════════════════════════════════════════════
    # NODE 1 — system_transformation (wall_detector)
    # ═══════════════════════════════════════════════════════════════════════
    system_transformation_node = Node(
        package    = 'vision',
        executable = 'system_transformation',
        name       = 'system_transformation_node',
        output     = 'screen',
        arguments  = ['--ros-args', '--log-level', log_level],
        parameters = [{
            'camera_pitch_deg':  camera_pitch,
            'image_width':       bbox_w,
            'image_height':      bbox_h,
            'ransac_dist':       ransac_dist,
            'ransac_iterations': ransac_iter,
            'min_inlier_ratio':  min_inliers,
            'padding_ratio':     padding,
            'sync_tolerance':    sync_tol,
            'image_tolerance':   img_tol,
            'buffer_size':       buf_size,
            'plane_sim_thresh':  plane_sim,
            'plane_dist_thresh': plane_dist,
            'objects_topic':     objects_topic,
        }],
    )

    # ═══════════════════════════════════════════════════════════════════════
    # NODE 2 — scene_descriptor
    # ═══════════════════════════════════════════════════════════════════════
    scene_descriptor_node = TimerAction(
        period  = 1.0,
        actions = [Node(
            package    = 'vision',
            executable = 'scene_description',
            name       = 'scene_descriptor_node',
            output     = 'screen',
            arguments  = ['--ros-args', '--log-level', log_level],
            parameters = [{'drone_heading_deg': heading, 'objects_topic': objects_topic}],
        )],
    )

    # ═══════════════════════════════════════════════════════════════════════
    # NODE 3 — overlay_node
    # ═══════════════════════════════════════════════════════════════════════
    overlay_node = TimerAction(
        period  = 2.0,
        actions = [Node(
            package    = 'vision',
            executable = 'image_overlay',
            name       = 'overlay_node',
            output     = 'screen',
            arguments = [
            '--ros-args', '--log-level', log_level,
            '-p', ['objects_topic:=', objects_topic],
            '-p', ['image_width:=',   bbox_w],
            '-p', ['image_height:=',  bbox_h],
            ],
            parameters = [{
                'objects_topic':    objects_topic,
                'scene_text_topic': scene_text_topic,
                'image_width':      bbox_w,
                'image_height':     bbox_h,
                'buffer_size':      buf_size,
                'save_images':      save_images,
                'save_dir':         save_dir,
                'max_saved':        max_saved,
            }],
        )],
    )

    # ═══════════════════════════════════════════════════════════════════════
    startup_msg = LogInfo(msg=(
        '\n'
        '╔════════════════════════════════════════════════════════════════════╗\n'
        '║         AEAC Vision Pipeline — démarrage                           ║\n'
        '╠════════════════════════════════════════════════════════════════════╣\n'
        '║  [0.0s] system_transformation                                      ║\n'
        '║  [1.0s] scene_descriptor                                           ║\n'
        '║  [2.0s] overlay_node                                               ║\n'
        '╠════════════════════════════════════════════════════════════════════╣\n'
        '║  Topics de sortie :                                                ║\n'
        '║    /aeac/internal/scene_description   (JSON system_transformation) ║\n'
        '║    /aeac/internal/scene_text          (texte scene_descriptor)     ║\n'
        '║    /aeac/external/overlay_image  (image annotée)                   ║\n'
        '║    /aeac/external/scene_frame    (message final ground stn)        ║\n'
        '╚════════════════════════════════════════════════════════════════════╝'
    ))

    return LaunchDescription(args + [
        startup_msg,
        system_transformation_node,
        scene_descriptor_node,
        overlay_node,
    ])