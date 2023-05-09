import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

PUZZLEBOT_SIM_PATH = get_package_share_directory('puzzlebot_sim')

RVIZ_PATH = get_package_share_directory('rviz2')
RVIZ_CONFIG = os.path.join(PUZZLEBOT_SIM_PATH, 'simulation_config.rviz')

def generate_launch_description():
    node_rviz_puzzlebot = Node(
        package='rviz2', executable='rviz2', output='screen',
        parameters=[{
            'display_config': RVIZ_CONFIG
        }],
    )

    node_pose_calculation = Node(
        package='puzzlebot_sim', executable='pose_calculation',
        output='screen'
    )

    node_tf_puzzlebot = Node(
        package='puzzlebot_sim', executable='puzzlebot_tf',
        output='screen'
    )

    node_data_gathering = Node (
        package='puzzlebot_sim', executable='data_gathering',
        output='screen'
    )
    
    return LaunchDescription([
        # node_rviz_puzzlebot,
        node_pose_calculation,
        node_tf_puzzlebot,
        # node_data_gathering,
    ])

