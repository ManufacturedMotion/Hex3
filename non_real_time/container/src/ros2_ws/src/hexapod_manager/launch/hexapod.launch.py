from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _load_robot_name():
    package_config_dir = os.path.join(
        get_package_share_directory('hexapod_manager'),
        'config'
    )
    local_config = os.path.join(package_config_dir, 'robot_name.local.txt')

    if os.path.exists(local_config):
        with open(local_config, 'r', encoding='utf-8') as f:
            name = f.read().strip()
            if name:
                return name

    return None


def _get_default_domain_id(robot_name):
    domain_map = {
        'Hewey': '10',
        'Dewey': '20',
        'Louie': '30',
    }
    return domain_map.get(robot_name, '0')


def generate_launch_description():

    robot_name = _load_robot_name() or 'default'
    domain_id = _get_default_domain_id(robot_name)

    os.environ['ROS_DOMAIN_ID'] = domain_id

    print(f"Starting robot '{robot_name}' with ROS_DOMAIN_ID={domain_id}")

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
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
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