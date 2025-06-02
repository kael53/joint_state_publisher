from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Parse collision_skip_pairs as a list from a comma-separated string
    collision_skip_pairs_str = LaunchConfiguration('collision_skip_pairs').perform(context)
    collision_skip_pairs = collision_skip_pairs_str.split(',') if collision_skip_pairs_str else []

    # All other parameters must be resolved to Python types (not LaunchConfiguration) for OpaqueFunction
    trajectory_time_step = float(LaunchConfiguration('trajectory_time_step').perform(context))
    planning_timeout = float(LaunchConfiguration('planning_timeout').perform(context))
    base_link = str(LaunchConfiguration('base_link').perform(context))
    right_tip = str(LaunchConfiguration('right_tip').perform(context))
    left_tip = str(LaunchConfiguration('left_tip').perform(context))
    detection_topic = str(LaunchConfiguration('detection_topic').perform(context))
    selected_class_topic = str(LaunchConfiguration('selected_class_topic').perform(context))
    planner_type = str(LaunchConfiguration('planner_type').perform(context))

    return [
        Node(
            package='unitree_g1_dex3_stack',
            executable='ik_fcl_ompl_planner',
            name='ik_fcl_ompl_planner',
            output='screen',
            parameters=[{
                'trajectory_time_step': trajectory_time_step,
                'planning_timeout': planning_timeout,
                'base_link': base_link,
                'right_tip': right_tip,
                'left_tip': left_tip,
                'detection_topic': detection_topic,
                'selected_class_topic': selected_class_topic,
                'planner_type': planner_type,
                'collision_skip_pairs': collision_skip_pairs
            }]
        )
    ]

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
        DeclareLaunchArgument('collision_skip_pairs', default_value=''),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
