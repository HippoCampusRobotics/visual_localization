from hippo_common.launch_helper import (
    LaunchArgsDict,
    config_file_path,
    declare_vehicle_name_and_sim_time,
    launch_file_source,
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description)

    action = DeclareLaunchArgument(
        name='camera_name',
        default_value='vertical_camera',
        description='The name of the camera.',
    )
    launch_description.add_action(action)

    pkg = 'visual_localization'
    action = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=config_file_path(pkg, 'ekf_params.yaml'),
        description='Path to the EKF configuration .yaml file.',
    )
    launch_description.add_action(action)
    action = DeclareLaunchArgument(
        name='tag_poses_file',
        default_value=config_file_path(pkg, 'tag_poses.yaml'),
        description='Path to the tag poses .yaml file.',
    )
    launch_description.add_action(action)
    action = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=config_file_path(pkg, 'apriltag_config.yaml'),
        description='Path to the apriltag_config.yaml file.',
    )
    launch_description.add_action(action)
    action = DeclareLaunchArgument(
        name='use_apriltag_viz',
        default_value='true',
        description='Publish AprilTag detection overlay image.',
    )
    launch_description.add_action(action)
    action = DeclareLaunchArgument(
        name='use_tag_markers',
        default_value='false',
        description='Publish visualization markers for the configured AprilTag map.',
    )
    launch_description.add_action(action)

    pkg = 'hippo_common'
    default = config_file_path(pkg, 'transformations_bluerov_default.yaml')
    action = DeclareLaunchArgument(
        name='tf_vehicle_config_file',
        description='tf config file',
        default_value=default,
    )
    launch_description.add_action(action)


def include_visual_localization():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add('camera_name')
    args.add('ekf_config_file')
    args.add('tag_poses_file')
    args.add('apriltag_config_file')
    args.add('use_apriltag_viz')
    args.add('use_tag_markers')

    pkg = 'visual_localization'
    source = launch_file_source(pkg, 'top_localization.launch.py')
    return IncludeLaunchDescription(
        source,
        launch_arguments=args.items(),
    )


def add_tf_publisher_vehicle_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_common',
        namespace=LaunchConfiguration('vehicle_name'),
        executable='tf_publisher_vehicle_node',
        parameters=[args, LaunchConfiguration('tf_vehicle_config_file')],
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    actions = [
        include_visual_localization(),
        add_tf_publisher_vehicle_node(),
    ]
    for action in actions:
        launch_description.add_action(action)
    return launch_description
