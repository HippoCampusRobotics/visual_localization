from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)
from launch_ros.actions import Node, PushROSNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)

    action = DeclareLaunchArgument(
        name='camera_name',
        default_value='vertical_camera',
        description='The name of the camera.',
    )
    launch_description.add_action(action)

    pkg_path = get_package_share_path('visual_localization')
    default_path = str(pkg_path / 'config/ekf_params.yaml')
    action = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=default_path,
        description='Path to the ekf configuration .yaml file.',
    )
    launch_description.add_action(action)

    pkg_path = get_package_share_path('visual_localization')
    default_path = str(pkg_path / 'config/tag_poses.yaml')
    action = DeclareLaunchArgument(
        name='tag_poses_file',
        default_value=default_path,
        description='Path to the tag poses .yaml file',
    )
    launch_description.add_action(action)

    pkg_path = get_package_share_path('visual_localization')
    default_path = str(pkg_path / 'config/apriltag_config.yaml')
    action = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=default_path,
        description='Path to the apriltag_config.yaml file.',
    )
    launch_description.add_action(action)


def create_apriltag_viz_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args['line_thickness'] = 5
    args['alpha'] = 1.0
    return Node(
        package='apriltag_viz',
        executable='apriltag_viz',
        namespace=LaunchConfiguration('camera_name'),
        parameters=[args],
        emulate_tty=True,
        output='screen',
    )


def create_relay_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add('input_topic')
    args['output_topic'] = '/tf'
    args['input_topic'] = 'tag_transforms'
    return Node(
        package='topic_tools',
        executable='relay',
        namespace=LaunchConfiguration('camera_name'),
        parameters=[args],
        emulate_tty=True,
        output='screen',
    )


def create_ekf_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add('tag_poses_file')
    args.add('camera_name')
    return Node(
        package='visual_localization',
        executable='vision_ekf_node',
        namespace=LaunchConfiguration('camera_name'),
        parameters=[
            args,
            LaunchConfiguration('ekf_config_file'),
        ],
        emulate_tty=True,
        output='screen',
    )


def create_tag_markers_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add('tag_poses_file')
    return Node(
        package='hippo_common',
        executable='tag_markers_publisher',
        parameters=[args],
        emulate_tty=True,
        output='screen',
    )


def create_apriltag_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='apriltag_ros',
        executable='apriltag_node',
        namespace=LaunchConfiguration('camera_name'),
        name='apriltag_node',
        remappings=[
            ('/tf', 'tag_transforms'),
        ],
        parameters=[
            args,
            LaunchConfiguration('apriltag_config_file'),
            {'pose_method': 'solve_pnp'},
        ],
        output='screen',
        emulate_tty=True,
    )


def create_px4_bridge_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='visual_localization',
        executable='px4_bridge',
        name='px4_bridge_node',
        parameters=[
            args,
        ],
        output='screen',
        emulate_tty=True,
    )


def include_image_decoder_node():
    package_path = get_package_share_path('visual_localization')
    path = str(package_path / 'launch/image_decoder.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    image_decoder = IncludeLaunchDescription(
        source, launch_arguments=args.items()
    )
    return image_decoder


def include_image_rectification_node():
    package_path = get_package_share_path('visual_localization')
    path = str(package_path / 'launch/image_rectification.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return IncludeLaunchDescription(source, launch_arguments=args.items())


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    action = GroupAction(
        [
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            create_ekf_node(),
            create_apriltag_node(),
            create_tag_markers_node(),
            create_relay_node(),
            create_apriltag_viz_node(),
            create_px4_bridge_node(),
            include_image_decoder_node(),
            include_image_rectification_node(),
        ]
    )
    launch_description.add_action(action)

    return launch_description
