from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # Get the path to the rviz configuration file
  rviz_config_path = os.path.join(
    get_package_share_directory('dynablox_ros'),
    'config',
    'rviz',
    'default.rviz'
  )

  return LaunchDescription([
    # Define the rviz node
    Node(
      package='rviz2',  # 'rviz2' is used in ROS 2
      executable='rviz2',  # In ROS 2, the executable is 'rviz2'
      name='rviz',
      arguments=['-d', rviz_config_path]
    )
  ])
