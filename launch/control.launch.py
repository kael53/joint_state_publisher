from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    tactile_threshold = LaunchConfiguration('tactile_threshold').perform(context)
    return [
        Node(
            package='unitree_g1_dex3_stack',
            executable='joint_trajectory_executor',
            name='joint_trajectory_executor',
            output='screen'
        ),
        Node(
            package='unitree_g1_dex3_stack',
            executable='dex3_controller',
            name='dex3_controller_left',
            output='screen',
            parameters=[{'side': 'left', 'tactile_threshold': float(tactile_threshold)}]
        ),
        Node(
            package='unitree_g1_dex3_stack',
            executable='dex3_controller',
            name='dex3_controller_right',
            output='screen',
            parameters=[{'side': 'right', 'tactile_threshold': float(tactile_threshold)}]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tactile_threshold',
            default_value='10.2',
            description='Tactile threshold for both hands.'
        ),
        OpaqueFunction(function=launch_setup)
    ])
