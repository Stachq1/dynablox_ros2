from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # Declare the file_path argument with a default value
  file_path_arg = DeclareLaunchArgument(
    'file_path',
    default_value='/home/{}/dynablox_output/clouds.csv'.format(os.environ['USER']),
    description='Path to the CSV file to visualize'
  )

  # Set up the cloud visualizer node
  cloud_visualizer_node = Node(
    package='dynablox_ros',
    executable='cloud_visualizer_node',
    name='cloud_visualizer_node',
    output='screen',
    arguments=['--alsologtostderr'],
    parameters=[{
        'file_path': LaunchConfiguration('file_path'),
        'static_point_scale': 0.05,
        'dynamic_point_scale': 0.1,
        'out_of_bounds_color': [0.8, 0.8, 0.8, 1.0]
    }],
    remappings=[]
  )

  # Get the path to the RViz configuration file
  rviz_config_path = os.path.join(
    get_package_share_directory('dynablox_ros'),
    'config',
    'rviz',
    'cloud_visualizer.rviz'
  )

  # Set up the RViz node
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz',
    arguments=['-d', rviz_config_path],
    output='screen'
  )

  return LaunchDescription([
    file_path_arg,
    cloud_visualizer_node,
    rviz_node
  ])
