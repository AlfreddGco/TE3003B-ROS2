import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

import xacro

puzzlebot_gazebo_path = os.path.join(
    get_package_share_directory('puzzlebot_gazebo'))

puzzlebot_world_path = os.path.join(
    get_package_share_directory('puzzlebot_world'))

def get_puzzlebot_urdf():
    xacro_file = os.path.join(
        puzzlebot_gazebo_path, 'urdf', 'puzzlebot.xacro')
    puzzlebot_xacro = xacro.parse(open(xacro_file))
    xacro.process_doc(puzzlebot_xacro)
    return puzzlebot_xacro


def generate_launch_description():
    SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=puzzlebot_world_path + '/models'),
    SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH', value=puzzlebot_world_path + '/models'),

    gazebo_dir = get_package_share_directory('gazebo_ros')
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_dir, 'launch'), '/gzserver.launch.py'
        ]),
        launch_arguments = {
            'world': os.path.join(puzzlebot_world_path, 'worlds/track.sdf')
        }.items()
    )
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_dir, 'launch'), '/gzclient.launch.py'
        ]),
    )

    puzzlebot_urdf = get_puzzlebot_urdf()
 
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': puzzlebot_urdf.toxml()
        }]
    )

    node_robot_joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': puzzlebot_urdf.toxml()
        }]
    )

    spawn_puzzlebot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', '-entity', 'puzzlebot',
            '-x', '1', '-y', '1', '-z', '0'
        ],
        output='screen'
    )

    ros_distro = os.getenv('ROS_DISTRO')
    state_cmd = 'active' if ros_distro == 'humble' else 'start'

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
        '--set-state', state_cmd, 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
            '--set-state', state_cmd, 'effort_controllers'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_puzzlebot,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        gz_server, gz_client,
        node_robot_state_publisher,
        spawn_puzzlebot,
    ])

