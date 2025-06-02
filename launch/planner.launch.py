from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    args = [
        DeclareLaunchArgument('trajectory_time_step', default_value='0.05'),
        DeclareLaunchArgument('planning_timeout', default_value='1.0'),
        DeclareLaunchArgument('base_link', default_value='pelvis'),
        DeclareLaunchArgument('right_tip', default_value='right_hand_palm_link'),
        DeclareLaunchArgument('left_tip', default_value='left_hand_palm_link'),
        DeclareLaunchArgument('detection_topic', default_value='/detections'),
        DeclareLaunchArgument('selected_class_topic', default_value='/selected_detection_class'),
        DeclareLaunchArgument('planner_type', default_value='RRTConnect'),
        DeclareLaunchArgument('collision_skip_pairs', default_value='[]'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]
    return LaunchDescription(args + [
        Node(
            package='unitree_g1_dex3_stack',
            executable='ik_fcl_ompl_planner',
            name='ik_fcl_ompl_planner',
            output='screen',
            parameters=[{
                'trajectory_time_step': LaunchConfiguration('trajectory_time_step'),
                'planning_timeout': LaunchConfiguration('planning_timeout'),
                'base_link': LaunchConfiguration('base_link'),
                'right_tip': LaunchConfiguration('right_tip'),
                'left_tip': LaunchConfiguration('left_tip'),
                'detection_topic': LaunchConfiguration('detection_topic'),
                'selected_class_topic': LaunchConfiguration('selected_class_topic'),
                'planner_type': LaunchConfiguration('planner_type'),
                'collision_skip_pairs': LaunchConfiguration('collision_skip_pairs'),
                'log_level': LaunchConfiguration('log_level'),
            }]
        )
    ])
