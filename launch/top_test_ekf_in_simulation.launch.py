"""
This launch file starts a self-contained simulation setup for testing the
visual EKF with AprilTags on the tank floor.

It starts:
- Gazebo
- simulated floor AprilTags
- simulated vehicle with vertical camera
- Gazebo/ROS bridges
- fake state estimator for simulated odometry
- vehicle TF publisher
- vehicle actuator mixer
- optional path follower
- AprilTag detector
- visual EKF
- optional RViz/debug helpers

This launch file is only meant for simulation.
"""

from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import (
    LaunchArgsDict,
    config_file_path,
    create_camera_bridge,
    launch_file_source,
    required_valid_vehicle_type,
)
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EqualsSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription


def declare_launch_args(launch_description: LaunchDescription) -> None:
    hippo_sim_path = get_package_share_path('hippo_sim')
    visual_localization_path = get_package_share_path('visual_localization')

    action = DeclareLaunchArgument(
        name='vehicle_name',
        default_value='uuv00',
        description='Vehicle name used as top-level namespace.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='vehicle_type',
        default_value='hippocampus',
        description='Vehicle model to spawn. Valid values: hippocampus, bluerov.',
    )
    launch_description.add_action(action)

    action = required_valid_vehicle_type()
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use /clock from Gazebo.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='start_gui',
        default_value='false',
        description='Start the Gazebo GUI.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='world_file',
        default_value=str(hippo_sim_path / 'models/world/empty.sdf'),
        description='Gazebo world file.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='camera_name',
        default_value='vertical_camera',
        description='Camera used for detecting the tank-floor AprilTags.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='tag_poses_file',
        default_value=str(hippo_sim_path / 'config/tag_poses.yaml'),
        description=(
            'Tag-pose file used both for spawning the floor tags and for '
            'the EKF landmark map.'
        ),
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=str(
            visual_localization_path / 'config/apriltag_config.yaml'
        ),
        description='AprilTag detector parameter file.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='tag_size',
        default_value='0.075',
        description=(
            'AprilTag edge size used by apriltag_ros. Must match the '
            'simulated floor tags.'
        ),
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=str(visual_localization_path / 'config/ekf_params.yaml'),
        description='Visual EKF parameter file.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='use_ekf',
        default_value='true',
        description='Start the visual EKF node.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='use_path_follower',
        default_value='false',
        description='Start the path follower.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='publish_tag_markers',
        default_value='true',
        description='Publish RViz markers for the configured tag map.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='use_apriltag_viz',
        default_value='false',
        description='Publish AprilTag debug overlay image.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='relay_tag_detections_to_tf',
        default_value='false',
        description=(
            'Relay AprilTag detection transforms to /tf for RViz debugging. '
            'Not required by the EKF.'
        ),
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='spawn_x',
        default_value='1.3',
        description='Initial vehicle x position in map/world frame.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='spawn_y',
        default_value='1.0',
        description='Initial vehicle y position in map/world frame.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='spawn_z',
        default_value='-0.5',
        description='Initial vehicle z position in map/world frame.',
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        name='spawn_yaw',
        default_value='1.5708',
        description='Initial vehicle yaw in radians.',
    )
    launch_description.add_action(action)


###############################################################################
# Simulation
###############################################################################


def include_gazebo() -> IncludeLaunchDescription:
    args = LaunchArgsDict()
    args.add(['start_gui', 'world_file'])
    return IncludeLaunchDescription(
        launch_file_source('hippo_sim', 'start_gazebo.launch.py'),
        launch_arguments=args.items(),
    )


def include_floor_tags() -> IncludeLaunchDescription:
    args = LaunchArgsDict()
    args.add('tag_poses_file')
    return IncludeLaunchDescription(
        launch_file_source('hippo_sim', 'spawn_apriltag_floor.launch.py'),
        launch_arguments=args.items(),
    )


def create_simulation_actions() -> list:
    return [
        include_gazebo(),
        include_floor_tags(),
    ]


###############################################################################
# Vehicle simulation
###############################################################################


def vehicle_type_is(vehicle_type: str) -> IfCondition:
    return IfCondition(
        EqualsSubstitution(LaunchConfiguration('vehicle_type'), vehicle_type)
    )


def create_robot_description(model_path: str) -> Command:
    return Command(
        [
            'ros2 run hippo_sim create_robot_description.py ',
            '--input ',
            model_path,
            ' --mappings vehicle_name=',
            LaunchConfiguration('vehicle_name'),
            ' use_vertical_camera=true',
            ' use_front_camera=false',
            ' use_range_sensor=false',
            ' use_acoustic_modem=false',
        ]
    )


def create_vehicle_spawn_node(model_path: str, condition: IfCondition) -> Node:
    return Node(
        package='hippo_sim',
        executable='spawn',
        namespace=LaunchConfiguration('vehicle_name'),
        parameters=[
            {'robot_description': create_robot_description(model_path)}
        ],
        arguments=[
            '--param',
            'robot_description',
            '--remove_on_exit',
            'true',
            '--x',
            LaunchConfiguration('spawn_x'),
            '--y',
            LaunchConfiguration('spawn_y'),
            '--z',
            LaunchConfiguration('spawn_z'),
            '--Y',
            LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
        condition=condition,
    )


def create_vehicle_bridge_node() -> Node:
    return Node(
        package='hippo_sim',
        executable='bridge',
        namespace=LaunchConfiguration('vehicle_name'),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )


def create_fake_state_estimator_node() -> Node:
    return Node(
        package='hippo_sim',
        executable='fake_state_estimator',
        namespace=LaunchConfiguration('vehicle_name'),
        name='state_estimator',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )


def create_tf_publisher_node(config_file: str, condition: IfCondition) -> Node:
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_common',
        executable='tf_publisher_vehicle_node',
        namespace=LaunchConfiguration('vehicle_name'),
        parameters=[args, config_file],
        output='screen',
        emulate_tty=True,
        condition=condition,
    )


def create_actuator_mixer_node(
    config_file: str, condition: IfCondition
) -> Node:
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_control',
        executable='actuator_mixer_node',
        namespace=LaunchConfiguration('vehicle_name'),
        parameters=[
            args,
            config_file,
        ],
        output='screen',
        emulate_tty=True,
        condition=condition,
    )


def create_vehicle_simulation_actions() -> list:
    hippo_sim_path = get_package_share_path('hippo_sim')
    hippo_common_path = get_package_share_path('hippo_common')

    hippocampus_model = str(hippo_sim_path / 'models/hippo3/urdf/hippo3.xacro')
    bluerov_model = str(hippo_sim_path / 'models/bluerov/urdf/bluerov.xacro')

    hippocampus_tf_config = str(
        hippo_common_path / 'config/transformations_hippo_default.yaml'
    )
    bluerov_tf_config = str(
        hippo_common_path / 'config/transformations_bluerov_default.yaml'
    )
    hippocampus_mixer_config = config_file_path(
        'hippo_control',
        'actuator_mixer/hippocampus_normalized_default.yaml',
    )

    return [
        create_camera_bridge(
            vehicle_name=LaunchConfiguration('vehicle_name'),
            camera_name=LaunchConfiguration('camera_name'),
            use_camera='true',
            image_name='image_rect',
        ),
        create_vehicle_spawn_node(
            hippocampus_model,
            vehicle_type_is('hippocampus'),
        ),
        create_vehicle_spawn_node(
            bluerov_model,
            vehicle_type_is('bluerov'),
        ),
        create_vehicle_bridge_node(),
        create_fake_state_estimator_node(),
        create_tf_publisher_node(
            hippocampus_tf_config,
            vehicle_type_is('hippocampus'),
        ),
        create_tf_publisher_node(
            bluerov_tf_config,
            vehicle_type_is('bluerov'),
        ),
        create_actuator_mixer_node(
            hippocampus_mixer_config,
            vehicle_type_is('hippocampus'),
        ),
    ]


###############################################################################
# Optional control stack
###############################################################################


def include_path_follower() -> IncludeLaunchDescription:
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return IncludeLaunchDescription(
        launch_file_source(
            'hippo_control',
            'top_path_following_intra_process.launch.py',
        ),
        launch_arguments=args.items(),
        condition=IfCondition(LaunchConfiguration('use_path_follower')),
    )


###############################################################################
# Visual localization pipeline
###############################################################################


def create_apriltag_node() -> Node:
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
            {
                # Old HippoCampus apriltag_ros fork.
                # For upstream apriltag_ros, remove this parameter and use
                # pose_estimation_method: pnp in apriltag_config.yaml.
                'pose_method': 'solve_pnp',
                'size': ParameterValue(
                    LaunchConfiguration('tag_size'),
                    value_type=float,
                ),
            },
        ],
        output='screen',
        emulate_tty=True,
    )


def create_ekf_node() -> Node:
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add(['camera_name', 'tag_poses_file'])
    return Node(
        package='visual_localization',
        executable='vision_ekf_node',
        namespace=LaunchConfiguration('camera_name'),
        parameters=[args, LaunchConfiguration('ekf_config_file')],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )


def create_tag_markers_node() -> Node:
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args.add('tag_poses_file')
    return Node(
        package='hippo_common',
        executable='tag_markers_publisher',
        parameters=[args],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('publish_tag_markers')),
    )


def create_apriltag_viz_node() -> Node:
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args['line_thickness'] = 5
    args['alpha'] = 1.0

    return Node(
        package='apriltag_viz',
        executable='apriltag_viz',
        namespace=LaunchConfiguration('camera_name'),
        name='apriltag_viz',
        parameters=[args],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_apriltag_viz')),
    )


def create_relay_node() -> Node:
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args['input_topic'] = 'tag_transforms'
    args['output_topic'] = '/tf'
    return Node(
        package='topic_tools',
        executable='relay',
        namespace=LaunchConfiguration('camera_name'),
        parameters=[args],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            LaunchConfiguration('relay_tag_detections_to_tf')
        ),
    )


def create_visual_localization_group() -> GroupAction:
    return GroupAction(
        [
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            create_apriltag_node(),
            create_ekf_node(),
            create_tag_markers_node(),
            create_relay_node(),
            create_apriltag_viz_node(),
        ]
    )


###############################################################################
# Launch description
###############################################################################


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_args(launch_description)

    actions = [
        *create_simulation_actions(),
        *create_vehicle_simulation_actions(),
        include_path_follower(),
        create_visual_localization_group(),
    ]

    for action in actions:
        launch_description.add_action(action)

    return launch_description
