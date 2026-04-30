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
