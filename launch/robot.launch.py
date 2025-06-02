from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_name_arg = DeclareLaunchArgument(
        'urdf_name',
        default_value='g1_29dof_lock_waist_with_hand_rev_1_0.urdf',
        description='URDF filename to load from robots/g1_description/'
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description='Full path to URDF file (overrides urdf_name if set)'
    )

    def get_robot_description(context):
        urdf_path_value = context.launch_configurations.get('urdf_path', '')
        if urdf_path_value:
            resolved_urdf_path = urdf_path_value
        else:
            urdf_path = os.path.join(
                get_package_share_directory('unitree_g1_dex3_stack'),
                'robots',
                'g1_description',
                context.launch_configurations.get('urdf_name', 'g1_29dof_lock_waist_with_hand_rev_1_0.urdf')
            )
            resolved_urdf_path = urdf_path
        with open(resolved_urdf_path, 'r') as infp:
            return {'robot_description': infp.read()}

    return LaunchDescription([
        urdf_name_arg,
        urdf_path_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[get_robot_description]
        ),
        Node(
            package='unitree_g1_dex3_stack',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    ])
