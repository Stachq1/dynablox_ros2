from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch import actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # Declare the arguments
  bag_file_arg = DeclareLaunchArgument(
    'bag_file',
    default_value='/home/spiasecki/Desktop/rosbags/indoor.bag',
    description='Full path to the bag file to play'
  )

  player_rate_arg = DeclareLaunchArgument(
    'player_rate',
    default_value='1',
    description='Real time rate of bag being played'
  )

  evaluate_arg = DeclareLaunchArgument(
    'evaluate',
    default_value='false',
    description='Whether to save evaluation data'
  )

  eval_output_path_arg = DeclareLaunchArgument(
    'eval_output_path',
    default_value='/home/{}/dynablox_output'.format(os.environ['USER']),
    description='Where to save evaluation data'
  )

  ground_truth_file_arg = DeclareLaunchArgument(
    'ground_truth_file',
    default_value='/home/{}/data/DOALS/hauptgebaeude/sequence_1/indices.csv'.format(os.environ['USER']),
    description='GT data file. Currently supports DOALS'
  )

  config_file_arg = DeclareLaunchArgument(
    'config_file',
    default_value='motion_detector/default.yaml',
    description='Configuration of Dynablox'
  )

  visualize_arg = DeclareLaunchArgument(
    'visualize',
    default_value='true',
    description='Whether to display RVIZ visualizations'
  )

  # Include play_dynablox_data.launch
  play_dynablox_data_launch = IncludeLaunchDescription(
    actions.PushSubscription(
      get_package_share_directory('dynablox_ros'),
      'launch/play_dynablox_data.launch',
      pass_all_args=True
    )
  )

  motion_detector_node = Node(
    package='dynablox_ros',
    executable='motion_detector',
    name='motion_detector',
    output='screen',
    arguments=['--alsologtostderr'],
    parameters=[
      {
        'evaluation/ground_truth/file_path': LaunchConfiguration('ground_truth_file'),
        'evaluation/output_directory': LaunchConfiguration('eval_output_path'),
        'evaluate': LaunchConfiguration('evaluate'),
        'config_file': os.path.join(
            get_package_share_directory('dynablox_ros'),
            'config',
            LaunchConfiguration('config_file')
        )
      }
    ],
    remappings=[]
  )

  # Include visualizer.launch if visualize is true
  visualizer_launch = IncludeLaunchDescription(
    actions.PushSubscription(
      get_package_share_directory('dynablox_ros'),
      'launch/visualizer.launch',
      condition=IfCondition(LaunchConfiguration('visualize'))
    )
  )

  return LaunchDescription([
    bag_file_arg,
    player_rate_arg,
    evaluate_arg,
    eval_output_path_arg,
    ground_truth_file_arg,
    config_file_arg,
    visualize_arg,
    play_dynablox_data_launch,
    motion_detector_node,
    visualizer_launch
  ])
