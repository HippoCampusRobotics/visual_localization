from ament_index_python.packages import get_package_share_path
import launch
import launch_ros
import os


def generate_launch_description():
    package_name = 'visual_localization'
    package_path = get_package_share_path(package_name)
    default_ekf_params_path = package_path / ('config/ekf_params.yaml')
    default_tag_poses_path = package_path / ('config/tag_poses.yaml')

    default_vehicle_name = 'uuv00'

    launch_args = [
        launch.actions.DeclareLaunchArgument(
            name='camera_name',
            default_value='vertical_camera',
            description='The name of the camera.',
        ),
        launch.actions.DeclareLaunchArgument(
            name='vehicle_name',
            default_value=default_vehicle_name,
            description='Vehicle name used as top level namespace.',
        ),
        launch.actions.DeclareLaunchArgument(
            name='ekf_params_path',
            default_value=str(default_ekf_params_path),
            description='Path to the mixer configuration .yaml file.'),
        launch.actions.DeclareLaunchArgument(
            name='tag_poses_path',
            default_value=str(default_tag_poses_path),
            description='Path to the tag poses .yaml file.'),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value=str(False),
        ),
    ]

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    apriltag_settings = os.path.join(get_package_share_path('apriltag_ros'),
                                     'config', 'settings.param.yaml')

    tags_standalone_path = os.path.join(
        get_package_share_path('visual_localization'), 'config',
        'tags_standalone.yaml')

    ekf_node = launch_ros.actions.Node(
        package=package_name,
        executable='vision_ekf_node',
        parameters=[
            launch.substitutions.LaunchConfiguration(
                'ekf_params_path'),  # load yaml file with ekf parameters
            {
                'tag_poses_path':
                launch.substitutions.LaunchConfiguration('tag_poses_path'),
                'camera_name':
                launch.substitutions.LaunchConfiguration('camera_name'),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    apriltag_node = launch_ros.actions.Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_detector_node',
        name='apriltag_node',
        remappings=[('~/image_rect', 'vertical_camera/image_rect'),
                    ('~/camera_info', 'vertical_camera/camera_info'),
                    ('~/tag_detections', 'tag_detections')],
        parameters=[
            apriltag_settings,
            tags_standalone_path,
            {
                'publish_tag_detections_image': True,
                'use_sim_time': use_sim_time,
            },
        ],
        output='screen',
    )

    nodes_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('vehicle_name')),
        apriltag_node,
        ekf_node,
    ])

    return launch.LaunchDescription(launch_args + [
        nodes_group,
    ])
