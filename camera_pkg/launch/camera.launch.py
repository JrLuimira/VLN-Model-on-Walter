import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'serial_test'
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'camera_rgbd.rviz'
    )

    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Namespace base para la cámara RealSense'
    )

    camera_name = LaunchConfiguration('camera_name')

    # Nodo de la cámara
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=camera_name,
        name='rs_d435',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 1,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 1,
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'pointcloud.stream_filter': 2,
            'pointcloud.stream_index_filter': 0,
            'pointcloud.ordered_pc': True,
            'publish_tf': True,
            'tf_publish_rate': 10.0,
            'enable_sync': True,
            'filters': "hole_filling",
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        declare_camera_name,
        camera_node,
        rviz_node
    ])
