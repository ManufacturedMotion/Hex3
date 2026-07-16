from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ik_config_file = os.path.join(
        get_package_share_directory('inverse_kinematics'),
        'config',
        'ik_config.json'
    )

    can_config_file = os.path.join(
        get_package_share_directory('can'),
        'config',
        'leg_groups.json'
    )

    control = Node(
        package='control',
        executable='xbox_joy_config',
        name='control',
        output='screen'
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
    )

    # Inverse kinematics
    inverse_kinematics = Node(
        package='inverse_kinematics',
        executable='inverse_kinematics',
        name='inverse_kinematics',
        output='screen',
        parameters=[
            {
                'ik_config' : ik_config_file
            }
        ],
    )

    # CAN interface
    can_interface = Node(
        package='can',
        executable='can_interface',
        name='can_interface',
        output='screen',
        parameters=[
            {
                'leg_groups_config' : can_config_file
            }
        ],
    )

    # Tripod Gait
    tripod_gait = Node(
        package='tripod_gait',
        executable='tripod_gait',
        name='tripod_gait',
        output='screen',
    )

    return LaunchDescription([
        inverse_kinematics,
        can_interface,
        tripod_gait,
        control,
        joy
    ])