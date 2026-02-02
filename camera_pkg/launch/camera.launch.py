import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'camera_pkg'
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'camera_rgbd.rviz'
    )

    owlvit_path = os.path.join(
        get_package_share_directory("walter_vln_model"),
        "nodes",
        "owlvit_node.py"   
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
             'color_width': 1280,   
            'color_height': 720, 
           # 'color_width': 640,   
           # 'color_height': 480,  
            'color_fps': 30.0,     
            
            'depth_width': 1280,   
            'depth_height': 720,
            'depth_fps': 30.0,
            
            'align_depth.enable': True, 
            
            'pointcloud.enable': False, 
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

      # === NODO 2: OWLVIT (se lanza DESPUÉS de la cámara) ===
    owlvit_node = Node(
        package="walter_vln_model",
        executable="owlvit_node.py",
        name="owlvit_node",
        output="screen"
    )
    yolov8_node = Node(
        package="walter_vln_model",
        executable="yolov8.py",
        name="yolo_node",
        output="screen"
    )
    qr_reader_node = Node(
        package="walter_vln_model",
        executable="qr_reader_node.py",
        name="qr_reader_node",
        output="screen"
    )

    # Handler para ejecutar OwlViT SOLO cuando la cámara ya arrancó
    start_owlvit_after_camera = RegisterEventHandler(
        OnProcessStart(
            target_action=camera_node,
            on_start=[owlvit_node]
        )
    )
    start_yolo_after_camera = RegisterEventHandler(
        OnProcessStart(
            target_action=camera_node,
            on_start=[yolov8_node]
        )
    )
    start_qr_after_camera = RegisterEventHandler(
        OnProcessStart(
            target_action=camera_node,
            on_start=[qr_reader_node]
        )
    )


    # Handler para ejecutar RVIZ solo cuando OwlViT está corriendo
    start_rviz_after_owlvit = RegisterEventHandler(
        OnProcessStart(
            target_action=owlvit_node,
            on_start=[rviz_node]
        )
    )

    return LaunchDescription([
        declare_camera_name,
        camera_node,
        #rviz_node,
        start_yolo_after_camera,
        #start_qr_after_camera
        #start_rviz_after_owlvit
    ])
