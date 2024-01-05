from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_description = LaunchDescription()

    action = Node(executable='image_decoder',
                  name='image_decoder',
                  package='visual_localization',
                  namespace=LaunchConfiguration('camera_name'),
                  parameters=[
                      {
                          'use_sim_time': False,
                      },
                  ])
    launch_description.add_action(action)

    return launch_description
