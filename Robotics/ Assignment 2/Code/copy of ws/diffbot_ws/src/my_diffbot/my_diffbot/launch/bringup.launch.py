from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_diffbot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'diffbot.urdf.xacro')

    # Process xacro to URDF
    robot_description = Command(['xacro ', xacro_file])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join('/opt/ros/jazzy/share/ros_gz_sim/launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'world': ''}.items()
    )

    # Robot State Publisher - publishes the robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn entity - uses topic instead of file
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'diffbot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        Node(
            package='my_diffbot',
            executable='cmdvel_to_wheels',
            name='cmdvel_to_wheels',
            output='screen'
        ),
    ])
