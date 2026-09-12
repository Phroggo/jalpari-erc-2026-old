"""Official evaluation interface. Simulation must already be running."""
from pathlib import Path
import json
import os
import time
import warnings
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def mark_trial_start(context):
    # Optional display observer reads this; no ROS/physics configuration changes.
    path = Path('/tmp') / ('erc_trial_start_' + os.environ.get('ROS_DOMAIN_ID', '0') + '.json')
    temporary = path.with_suffix('.tmp')
    try:
        temporary.write_text(json.dumps({'monotonic_ns': time.monotonic_ns()}))
        temporary.replace(path)
    except OSError as exc:
        warnings.warn(f'Optional recording timer unavailable: {exc}')
    return []


def generate_launch_description():
    source = Path(__file__).resolve()
    repository = source.parents[3] if source.parents[2].name == 'src' else Path.cwd()
    return LaunchDescription([
        DeclareLaunchArgument('shelf_column_number',default_value='2'),
        DeclareLaunchArgument('book_colour',default_value='red'),
        DeclareLaunchArgument('images_dir',default_value=str(repository/'erc_images')),
        OpaqueFunction(function=mark_trial_start),
        Node(package='erc_solution',executable='book_retrieval',output='screen',
             parameters=[{'use_sim_time':True,
                          'shelf_column_number':ParameterValue(LaunchConfiguration('shelf_column_number'),value_type=int),
                          'book_colour':LaunchConfiguration('book_colour'),
                          'images_dir':LaunchConfiguration('images_dir')}]),
    ])
