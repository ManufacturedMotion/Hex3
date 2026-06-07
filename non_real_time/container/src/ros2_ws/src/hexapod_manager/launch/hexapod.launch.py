from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Gait planner
    gait_planner = Node(
        package='hexapod_control',
        executable='gait_planner',
        name='gait_planner',
        output='screen'
    )

    # Inverse kinematics
    inverse_kinematics = Node(
        package='hexapod_control',
        executable='inverse_kinematics',
        name='inverse_kinematics',
        output='screen'
        parameters=[
            {
                'leg_groups_config' :   
            }
        ]
    )

    # CAN interface
    can_interface = Node(
        package='can',
        executable='can_interface',
        name='can_interface',
        output='screen'
    )
    extra_node = Node(
        package='some_package',
        executable='some_executable',
        name='some_node',
        output='screen'
    )

    return LaunchDescription([
        gait_planner,
        inverse_kinematics,
        can_interface,

        # Uncomment to enable
        # extra_node,
    ])