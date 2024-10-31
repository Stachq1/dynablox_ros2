from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
  # Declare the bag_file argument with a default value of an empty string
  bag_file_arg = DeclareLaunchArgument(
    'bag_file',
    default_value='',
    description='Path to rosbag file to be played'
  )

  # Declare the player_rate argument with a default value of "1"
  player_rate_arg = DeclareLaunchArgument(
    'player_rate',
    default_value='1',
    description='Realtime factor to play data'
  )

  # Define the ros2 bag play command
  play_bag = ExecuteProcess(
    cmd=[
      'ros2', 'bag', 'play',
      LaunchConfiguration('bag_file'),
      '--rate', LaunchConfiguration('player_rate'),
      '--quiet',
      '-d', '1'
    ],
    output='screen',
    shell=False
  )

  return LaunchDescription([
    bag_file_arg,
    player_rate_arg,
    play_bag
  ])
