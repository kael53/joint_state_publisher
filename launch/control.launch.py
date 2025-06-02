from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
            parameters=[{'side': 'left'}]
        ),
        Node(
            package='unitree_g1_dex3_stack',
            executable='dex3_controller',
            name='dex3_controller_right',
            output='screen',
            parameters=[{'side': 'right'}]
        )
    ])
