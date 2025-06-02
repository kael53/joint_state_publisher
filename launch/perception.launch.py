from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    yolox_model_path_arg = DeclareLaunchArgument(
        'yolox_model_path',
        default_value='./install/yolox_ros_cpp/share/yolox_ros_cpp/weights/tensorrt/yolox_s.trt',
        description='Path to YOLOX TensorRT model file'
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/objects_3d',
        description='Output topic for projected 3D point cloud'
    )
    detection3d_topic_arg = DeclareLaunchArgument(
        'detection3d_topic',
        default_value='/detections_3d',
        description='Output topic for Detection3DArray'
    )
    output_frame_arg = DeclareLaunchArgument(
        'output_frame',
        default_value='d435_link',
        description='Output frame for projected data'
    )
    allowed_classes_arg = DeclareLaunchArgument(
        'allowed_classes',
        default_value="['cup','bottle','book','bowl']",
        description='Allowed classes for projection node (as Python list string, or empty for all)'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
        }.items()
    )

    yolox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolox_ros_cpp'),
                'launch',
                'yolox_tensorrt_jetson.launch.py'
            )
        ),
        launch_arguments={
            'imshow_isshow': 'false',
            'src_image_topic_name': '/camera/color/image_raw',
            'model_path': LaunchConfiguration('yolox_model_path'),
        }.items()
    )

    # Parse allowed_classes string as Python list in the node (user can pass '' for all)
    project_to_3d_node = Node(
        package='unitree_g1_dex3_stack',
        executable='project_to_3d_node',
        name='project_to_3d_node',
        output='screen',
        parameters=[{
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'detections_topic': '/yolox/bounding_boxes',
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'detection3d_topic': LaunchConfiguration('detection3d_topic'),
            'output_frame': LaunchConfiguration('output_frame'),
            'allowed_classes': LaunchConfiguration('allowed_classes'),
        }]
    )

    return LaunchDescription([
        yolox_model_path_arg,
        pointcloud_topic_arg,
        detection3d_topic_arg,
        output_frame_arg,
        allowed_classes_arg,
        realsense_launch,
        yolox_launch,
        project_to_3d_node
    ])
