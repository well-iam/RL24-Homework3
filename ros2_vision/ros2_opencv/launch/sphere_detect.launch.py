import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    iiwa_launch_path = os.path.join(
            FindPackageShare('iiwa_bringup').find('iiwa_bringup'), 'launch', 'iiwa.launch.py'
        )

    iiwa_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(iiwa_launch_path),
            launch_arguments={
                'gazebo_world': 'sphere.world',
                'use_vision': 'true',
                'initial_positions_file': 'initial_positions_sphere.yaml'
                
            }.items()
        )

    ros2_opencv = Node(
            package='ros2_opencv',   # The package name
            executable='ros2_opencv_node',   # The node name
            name='ros2_opencv_node',         # The name for the node
        )

    rqt_image_view = Node(
            package='rqt_image_view',   # The package name
            executable='rqt_image_view',   # The node name
            name='rqt_image_view',         # The name for the node
            arguments=['/processed_image']
        )
    
    delay_rqt_after_opencv = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_opencv,
            on_start=[rqt_image_view],
        )
    )
    return LaunchDescription(
        [
           iiwa_launch,
           ros2_opencv, 
           delay_rqt_after_opencv
         
        ]
    )