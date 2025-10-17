import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_file = os.path.join(
      get_package_share_directory('mynt_eye_ros2_wrapper'),
      'config',
      'config.yaml')

    return LaunchDescription([
          Node(
              package='mynt_eye_ros2_wrapper',      # your package name
              executable='mynt_eye_ros2_wrapper',      # your node executable
              name='mynt_eye_ros2_wrapper',            # node name
              output='screen',
              parameters=[param_file]    # load parameters from YAML
          )
      ])
