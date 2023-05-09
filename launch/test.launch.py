from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_name = 'visual_localization'
    package_path = get_package_share_path(package_name)
    default_ekf_params_path = package_path / ('config/ekf_params.yaml')
    default_tag_poses_path = package_path / ('config/tag_poses.yaml')

    ekf_launch_arg = launch.actions.DeclareLaunchArgument(
        name='ekf_params_path',
        default_value=str(default_ekf_params_path),
        description='Path to the mixer configuration .yaml file.')

    tag_poses_path_arg = launch.actions.DeclareLaunchArgument(
        name='tag_poses_path',
        default_value=str(default_tag_poses_path),
        description='Path to the tag poses .yaml file.')

    return launch.LaunchDescription([
        ekf_launch_arg,
        tag_poses_path_arg,
        launch_ros.actions.Node(
            package=package_name,
            executable='vision_ekf_node',
            parameters=[
                launch.substitutions.LaunchConfiguration('ekf_params_path'),
                {
                    'tag_poses_path':
                    launch.substitutions.LaunchConfiguration('tag_poses_path'),
                },
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
